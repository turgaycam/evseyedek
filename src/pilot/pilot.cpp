#include "pilot.h"
#include "app_pins.h"
#include "app_config.h"
#include <Preferences.h>
#include "app_log.h"
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

static constexpr float kCpTuneMinRatio = 2.0f;
static constexpr float kCpTuneMaxRatio = 8.0f;
static constexpr float kCpNomA = 12.0f;
static constexpr float kCpTuneMinAdc = 1.80f;
static constexpr float kCpTuneMaxAdc = 3.20f;
static constexpr float kCpTuneDcSpreadMax = 0.12f;
static constexpr uint8_t kCpTuneSamples = 6;
static constexpr uint32_t kCpTuneMinUptimeMs = 3000;
static constexpr uint32_t kCpTuneRetryMs = 4000;

static float s_tuneAdcSum = 0.0f;
static uint8_t s_tuneCount = 0;
static uint32_t s_lastTuneMs = 0;
static bool s_userLockedDivider = false;

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

static void autotuneDividerTo12v(float adcH, float adcL)
{
  if (s_userLockedDivider) return;
  if (pwmEnabled) {
    s_tuneAdcSum = 0.0f;
    s_tuneCount = 0;
    return;
  }
  if (measuredState != "A") {
    s_tuneAdcSum = 0.0f;
    s_tuneCount = 0;
    return;
  }
  if (millis() < kCpTuneMinUptimeMs) return;
  if (!isfinite(adcH) || !isfinite(adcL)) return;
  if (adcH < kCpTuneMinAdc || adcH > kCpTuneMaxAdc) return;
  if (fabsf(adcH - adcL) > kCpTuneDcSpreadMax) return;

  s_tuneAdcSum += adcH;
  s_tuneCount++;
  if (s_tuneCount < kCpTuneSamples) return;

  const float avgAdc = s_tuneAdcSum / (float)s_tuneCount;
  s_tuneAdcSum = 0.0f;
  s_tuneCount = 0;
  if (avgAdc < kCpTuneMinAdc || avgAdc > kCpTuneMaxAdc) return;

  const float next = clampDivider(kCpNomA / avgAdc);
  const float shown = avgAdc * CP_DIVIDER_RATIO;
  if (fabsf(shown - kCpNomA) <= 0.12f) return;
  if (fabsf(next - CP_DIVIDER_RATIO) < 0.008f) return;
  if (s_lastTuneMs != 0 && (millis() - s_lastTuneMs) < kCpTuneRetryMs) return;

  LOGI("[CP] Divider otomatik: %.3f -> %.3f (ADC %.3fV)",
       CP_DIVIDER_RATIO, next, avgAdc);
  CP_DIVIDER_RATIO = next;
  saveDivider(next);
  s_lastTuneMs = millis();
}

static bool validThresh(float v)
{
  return isfinite(v) && v >= 0.0f && v <= 15.0f;
}

static void loadCpUserCal()
{
  Preferences prefs;
  if (!prefs.begin("cpcfg", true)) return;

  float storedDiv = prefs.getFloat("div", 0.0f);
  if (storedDiv >= kCpTuneMinRatio && storedDiv <= kCpTuneMaxRatio && isfinite(storedDiv)) {
    CP_DIVIDER_RATIO = storedDiv;
  }

  float thb = prefs.getFloat("thb", -1.0f);
  float thc = prefs.getFloat("thc", -1.0f);
  float thd = prefs.getFloat("thd", -1.0f);
  float the = prefs.getFloat("the", -1.0f);
  float tha = prefs.getFloat("tha", -1.0f);
  float mUp = prefs.getFloat("mUp", -1.0f);
  float mDn = prefs.getFloat("mDn", -1.0f);
  int stb = prefs.getInt("stb", -1);

  if (validThresh(thb)) TH_B_MIN = thb;
  if (validThresh(thc)) TH_C_MIN = thc;
  if (validThresh(thd)) TH_D_MIN = thd;
  if (validThresh(the)) TH_E_MIN = the;
  if (validThresh(tha)) TH_A_MIN = tha;
  if (mUp >= 0.0f && mUp <= 2.0f) marginUp = mUp;
  if (mDn >= 0.0f && mDn <= 2.0f) marginDown = mDn;
  if (stb >= 1 && stb <= 50) stableCount = stb;

  prefs.end();
  LOGI("[CP] div=%.3f TH B/C/D/E=%.2f/%.2f/%.2f/%.2f stb=%d",
       CP_DIVIDER_RATIO, TH_B_MIN, TH_C_MIN, TH_D_MIN, TH_E_MIN, stableCount);
}

void pilot_set_divider(float ratio)
{
  CP_DIVIDER_RATIO = clampDivider(ratio);
  saveDivider(CP_DIVIDER_RATIO);
  s_userLockedDivider = true;
  s_tuneAdcSum = 0.0f;
  s_tuneCount = 0;
}

void pilot_save_state_thresholds()
{
  Preferences prefs;
  if (!prefs.begin("cpcfg", false)) return;
  prefs.putFloat("thb", TH_B_MIN);
  prefs.putFloat("thc", TH_C_MIN);
  prefs.putFloat("thd", TH_D_MIN);
  prefs.putFloat("the", TH_E_MIN);
  prefs.putFloat("tha", TH_A_MIN);
  prefs.putFloat("mUp", marginUp);
  prefs.putFloat("mDn", marginDown);
  prefs.putInt("stb", stableCount);
  prefs.end();
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
  loadCpUserCal();

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

  autotuneDividerTo12v(adcHigh, adcLow);

  cpHighVolt = adcHigh * CP_DIVIDER_RATIO;
  cpLowVolt  = adcLow  * CP_DIVIDER_RATIO;

  measuredStateRaw = decideStateHysteresis(cpHighVolt, measuredState);

  // State birkac ard arda okumada ayni gelirse stable kabul edilir.
  static int cnt = 0;
  static String last = "A";

  if (measuredStateRaw == last) cnt++;
  else { cnt = 1; last = measuredStateRaw; }

  if (cnt >= stableCount) measuredState = measuredStateRaw;
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
