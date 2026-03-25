#include "pilot.h"
#include "app_pins.h"
#include "app_config.h"

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
  const int N = 200;
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
