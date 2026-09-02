#include "pilot.h"
#include "app_pins.h"
#include "app_config.h"
#include <Preferences.h>
#include <math.h>

// CP state cikarma ve PWM uygulama modulu.
// Esik degerleri main.cpp'de tutulur, burada yorumlanir.

// PWM kontrol değişkenleri
bool pwmEnabled     = false;   // boot'ta kapalı başlayacağız
int  pwmDutyPercent = PWM_DUTY_32A; // sabit dursun (istersen 100 yaparsın)

// Ölçümler
static float  adcHigh = 0.0f, adcLow = 0.0f;
static float  cpHighVolt = 0.0f, cpLowVolt = 0.0f;
static String measuredStateRaw = "A";
static String measuredState    = "A";

static constexpr float kCpStateANomV = 12.0f;
static constexpr float kCpTuneMinV = 11.0f;
static constexpr float kCpTuneMaxV = 14.8f;
static constexpr float kCpTuneDeadbandV = 0.12f;
static constexpr float kCpTuneMinAdcV = 1.6f;
static constexpr float kCpTuneMaxAdcV = 3.3f;
static constexpr float kCpTuneMinRatio = 2.0f;
static constexpr float kCpTuneMaxRatio = 8.0f;
static constexpr uint8_t kCpTuneSamples = 10;
static constexpr uint32_t kCpTuneMinUptimeMs = 8000;
static constexpr uint32_t kCpTuneSaveMinMs = 30000;

static float s_tuneSum = 0.0f;
static uint8_t s_tuneCount = 0;
static uint32_t s_lastTuneSaveMs = 0;

static float clampDivider(float ratio)
{
  if (!isfinite(ratio)) return CP_DIVIDER_RATIO;
  if (ratio < kCpTuneMinRatio) return kCpTuneMinRatio;
  if (ratio > kCpTuneMaxRatio) return kCpTuneMaxRatio;
  return ratio;
}

static void saveDivider(float ratio)
{
  Preferences prefs;
  if (!prefs.begin("cpcfg", false)) return;
  prefs.putFloat("div", ratio);
  prefs.end();
}

static void loadDivider()
{
  Preferences prefs;
  if (!prefs.begin("cpcfg", true)) return;
  float stored = prefs.getFloat("div", 0.0f);
  prefs.end();
  if (stored >= kCpTuneMinRatio && stored <= kCpTuneMaxRatio && isfinite(stored)) {
    CP_DIVIDER_RATIO = stored;
    Serial.printf("[CP] Kayitli divider: %.3f\n", CP_DIVIDER_RATIO);
  }
}

void pilot_set_divider(float ratio)
{
  CP_DIVIDER_RATIO = clampDivider(ratio);
  saveDivider(CP_DIVIDER_RATIO);
  s_tuneSum = 0.0f;
  s_tuneCount = 0;
}

static void autotuneStateA(float cpHigh, float adcHigh)
{
  if (pwmEnabled) {
    s_tuneSum = 0.0f;
    s_tuneCount = 0;
    return;
  }
  if (measuredState != "A") {
    s_tuneSum = 0.0f;
    s_tuneCount = 0;
    return;
  }
  if (millis() < kCpTuneMinUptimeMs) return;
  if (!isfinite(cpHigh) || !isfinite(adcHigh)) return;
  if (adcHigh < kCpTuneMinAdcV || adcHigh > kCpTuneMaxAdcV) return;
  if (cpHigh < kCpTuneMinV || cpHigh > kCpTuneMaxV) return;

  const float errorV = fabsf(cpHigh - kCpStateANomV);
  if (errorV <= kCpTuneDeadbandV) {
    s_tuneSum = 0.0f;
    s_tuneCount = 0;
    return;
  }

  s_tuneSum += cpHigh;
  s_tuneCount++;
  if (s_tuneCount < kCpTuneSamples) return;

  const float avgV = s_tuneSum / (float)s_tuneCount;
  s_tuneSum = 0.0f;
  s_tuneCount = 0;
  if (avgV < kCpTuneMinV || avgV > kCpTuneMaxV) return;

  const float next = clampDivider(CP_DIVIDER_RATIO * (kCpStateANomV / avgV));
  if (fabsf(next - CP_DIVIDER_RATIO) < 0.01f) return;
  if ((millis() - s_lastTuneSaveMs) < kCpTuneSaveMinMs && s_lastTuneSaveMs != 0) return;

  Serial.printf("[CP] Otomatik divider: %.3f -> %.3f (olculen %.2fV -> 12.00V)\n",
                CP_DIVIDER_RATIO, next, avgV);
  CP_DIVIDER_RATIO = next;
  saveDivider(next);
  s_lastTuneSaveMs = millis();
}

// CP high seviyesine bakarak IEC state karari burada verilir.
// TH_* ve margin degerleri web panelinden degistirilince bu fonksiyonu etkiler.
static String decideStateHysteresis(float v, const String& cur)
{
  // A: v >= TH_A_MIN
  // B: TH_B_MIN <= v < TH_A_MIN
  // C: TH_C_MIN <= v < TH_B_MIN
  // D: TH_D_MIN <= v < TH_C_MIN
  // E: v < TH_D_MIN

  if (cur == "A") {
    if (v < (TH_A_MIN - marginDown)) return "B";
    return "A";
  }
  if (cur == "B") {
    if (v >= (TH_A_MIN + marginUp)) return "A";
    if (v < (TH_B_MIN - marginDown)) return "C";
    return "B";
  }
  if (cur == "C") {
    if (v >= (TH_B_MIN + marginUp)) return "B";
    if (v < (TH_C_MIN - marginDown)) return "D";
    return "C";
  }
  if (cur == "D") {
    if (v >= (TH_C_MIN + marginUp)) return "C";
    if (v < (TH_D_MIN - marginDown)) return "E";
    return "D";
  }
  // cur == "E" veya başka
  if (v >= (TH_D_MIN + marginUp)) return "D";
  return "E";
}

void pilot_init()
{
  loadDivider();

  // CP PWM cikisi ve ADC girisi ayni moduldedir.
  analogReadResolution(12);
  pinMode(CP_ADC_PIN, INPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(CP_PWM_PIN, PWM_CHANNEL);

  // Boot anında CP'yi HIGH'a çek (PWM OFF olsa bile A stabil olsun)
  ledcWrite(PWM_CHANNEL, 1023);
}

void pilot_apply_pwm()
{
  // PWM kapalıysa veya Duty 0 ise hattı sabit 12V'da (State A voltajı) tut
  if (!pwmEnabled || pwmDutyPercent <= 0) {
    // 10-bit çözünürlükte (0-1023) tam HIGH
    ledcWrite(PWM_CHANNEL, 1023); 
    return;
  }

  // PWM aktifse Duty Cycle'ı hesapla ve uygula
  // Duty cycle %53 ise -> 1023 * 0.53 = 542
  int dutyValue = (pwmDutyPercent * 1023) / 100;
  dutyValue = constrain(dutyValue, 0, 1023);
  
  ledcWrite(PWM_CHANNEL, dutyValue);
}


void pilot_update()
{
  // Basit min/max yaklasimi ile CP sinyalinin high/low seviyeleri olculur.
  // 1 kHz PWM periyodu ~1 ms; 120 ornek yaklasik bir periyodu kaplar.
  const int N = 120;
  int minRaw = 4095;
  int maxRaw = 0;

  for (int i = 0; i < N; i++) {
    int v = analogRead(CP_ADC_PIN);
    if (v < minRaw) minRaw = v;
    if (v > maxRaw) maxRaw = v;
  }

  adcHigh = (maxRaw * 3.3f) / 4095.0f;
  adcLow  = (minRaw * 3.3f) / 4095.0f;

  cpHighVolt = adcHigh * CP_DIVIDER_RATIO;
  cpLowVolt  = adcLow  * CP_DIVIDER_RATIO;

  measuredStateRaw = decideStateHysteresis(cpHighVolt, measuredState);

  // State birkac ard arda okumada ayni gelirse stable kabul edilir.
  static int cnt = 0;
  static String last = "A";

  if (measuredStateRaw == last) cnt++;
  else { cnt = 1; last = measuredStateRaw; }

  if (cnt >= stableCount) measuredState = measuredStateRaw;

  autotuneStateA(cpHighVolt, adcHigh);
}

PilotMeasurements pilot_get()
{
  PilotMeasurements m;
  m.adcHigh = adcHigh;
  m.adcLow  = adcLow;
  m.cpHigh  = cpHighVolt;
  m.cpLow   = cpLowVolt;
  m.stateRaw = measuredStateRaw;
  m.stateStable = measuredState;
  return m;
}
