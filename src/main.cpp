#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "app_pins.h"
#include "pilot/pilot.h"
#include "io/relay.h"
#include "net/web_ui.h"
#include "ui/oled_ui.h"
#include "io/current_sensor.h" // YENİ: Akım sensörü kütüphanesi eklendi

// Bu dosya projenin merkez akisidir.
// Neyi nereden degistirecegini hizli bulmak icin:
// - pin degisimi: include/app_pins.h
// - CP / state esikleri: bu dosyadaki global ayarlar
// - pilot state mantigi: src/pilot/pilot.cpp
// - role gecikmesi: src/io/relay.cpp
// - OLED gorunumu: src/ui/oled_ui.cpp
// - web panel ve API: src/net/web_ui.cpp

// CP yorumlama ayarlari. Web panelinden de guncellenebilir.
// web’den değişebilir
float CP_DIVIDER_RATIO = 4.62f;
float TH_A_MIN = 11.0f;
float TH_B_MIN = 9.6f;
float TH_C_MIN = 7.8f;
float TH_D_MIN = 5.0f;
float TH_E_MIN = 1.0f;
float marginUp = 0.30f;
float marginDown = 0.30f;
int   stableCount = 3;
int   loopIntervalMs = 200; // Varsayılan ölçüm hızı 200ms

float g_powerW = 0.0f;
float g_energyKWh = 0.0f;
uint32_t g_chargeSeconds = 0;
int g_phaseCount = 1;
float g_currentLimitA = 32.0f;

// Hızlı kontrol modu: 0=AUTO, 1=START, 2=STOP
int g_chargeMode = 0;
uint32_t g_manualStopAlertUntilMs = 0;
uint32_t g_manualStopAutoResumeAtMs = 0;

// Aktif seans bilgisi
bool g_sessionLive = false;
uint32_t g_sessionLiveStartSec = 0;
uint32_t g_sessionLiveSeconds = 0;
float g_sessionLiveEnergyKWh = 0.0f;

// Son seanslar (cihaz açık kaldığı sürece tutulur)
uint32_t g_histStartSec[20] = {0};
uint32_t g_histDurationSec[20] = {0};
float g_histEnergyKWh[20] = {0.0f};
float g_histAvgPowerW[20] = {0.0f};
uint8_t g_histPhaseCount[20] = {0};
int g_histCount = 0;
int g_histHead = 0;

static float energyWh = 0.0f;
static uint32_t lastEnergyMs = 0;
static uint32_t chargeAccumMs = 0;
static uint32_t chargeSegmentStartMs = 0;
static uint32_t sessionStartMs = 0;
static float sessionStartEnergyWh = 0.0f;
static uint8_t sessionPhaseMax = 1;

// Gecmis ekranini ve web API'sini sifirlamak icin kullanilir.
void resetHistoryData()
{
  for (int i = 0; i < 20; i++) {
    g_histStartSec[i] = 0;
    g_histDurationSec[i] = 0;
    g_histEnergyKWh[i] = 0.0f;
    g_histAvgPowerW[i] = 0.0f;
    g_histPhaseCount[i] = 0;
  }
  g_histCount = 0;
  g_histHead = 0;
}

void resetChargeData(bool clearHistory)
{
  energyWh = 0.0f;
  lastEnergyMs = 0;
  chargeAccumMs = 0;
  chargeSegmentStartMs = 0;
  sessionStartMs = 0;
  sessionStartEnergyWh = 0.0f;
  sessionPhaseMax = 1;

  g_powerW = 0.0f;
  g_energyKWh = 0.0f;
  g_chargeSeconds = 0;
  g_phaseCount = 1;

  g_sessionLive = false;
  g_sessionLiveStartSec = 0;
  g_sessionLiveSeconds = 0;
  g_sessionLiveEnergyKWh = 0.0f;

  if (clearHistory) {
    resetHistoryData();
  }
}


static bool lastPwmEnabled = false;
static int  lastDuty = -1;

// PWM yalnizca gercekten degisince yazilir; gereksiz ledc guncellemesi engellenir.
static void applyPwmIfChanged()
{
  if (pwmEnabled != lastPwmEnabled || pwmDutyPercent != lastDuty) {
    pilot_apply_pwm();
    lastPwmEnabled = pwmEnabled;
    lastDuty = pwmDutyPercent;
  }
}

// IEC 61851 yaklaşık dönüşüm (Teorik akım limiti hesabı)
static float duty_to_amps(int dutyPercent)
{
  if (dutyPercent <= 0) return 0.0f;

  if (dutyPercent < 10) {
    return 0.0f;
  } else if (dutyPercent <= 85) {
    return dutyPercent * 0.6f;
  } else if (dutyPercent <= 96) {
    return (dutyPercent - 64) * 2.5f;
  }
  return 0.0f;
}

void setup()
{
  // Baslatma sirasi bilincli tutuldu:
  // 1) web/OTA, 2) OLED, 3) sensorler, 4) role, 5) pilot
  Serial.begin(115200);
  delay(200);
  Serial.println("BOOT OK");

  pinMode(STATE_LED_PIN, OUTPUT);
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(STATE_LED_PIN, LOW);
  digitalWrite(WIFI_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);


  // Web + OTA
  web_init();

  // OLED
  oled_init();

  // YENİ: Akım Sensörünü Başlat
  current_sensor_init();

  // Röle
  relay_init();
  relay_set_auto_enabled(true);
  relay_set_min_switch_ms(100);   // C<->B testleri için

  // Test dirençleri

  // Pilot
  pilot_init();

  // PWM başlangıç
  pwmDutyPercent = PWM_DUTY_32A;  // örn: 53
  pwmEnabled = false;            // boot'ta A kabul, PWM kapalı
  applyPwmIfChanged();
}

void loop()
{
  // Ana dongu akisi:
  // 1) arka plan servisleri
  // 2) CP state olcumu
  // 3) enerji / seans hesabi
  // 4) ekran ve LED guncellemesi
  // 5) PWM ve role kararinin uygulanmasi

  // Web sunucu döngüsü
  web_loop();

  // YENİ: Akım sensörü döngüsü (Sürekli çalışmalı)
  current_sensor_loop();

  static uint32_t last = 0;
  int safeLoopIntervalMs = loopIntervalMs;
  if (safeLoopIntervalMs < 20) safeLoopIntervalMs = 20;
  if (safeLoopIntervalMs > 2000) safeLoopIntervalMs = 2000;
  if (millis() - last < (uint32_t)safeLoopIntervalMs) return;
  last = millis();

  pilot_update();
  auto m = pilot_get();

  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);

  // Kablo: A değilse takılı kabul
  bool cableConnected = (m.stateStable != "A");

  // Sensor verisi burada okunur. OLED ve web tarafina giden anlik akim kaynagi burasidir.
  float ia = current_sensor_get_irms_a();
  float ib = current_sensor_get_irms_b();
  float ic = current_sensor_get_irms_c();
  bool relaySet = relay_get();
  bool chargingState = (m.stateStable == "C" || m.stateStable == "D");
  bool accountingEnabled = relaySet && pwmEnabled && chargingState;

  // Enerji ve akim sadece role cekili + PWM aktif + state C/D oldugunda gecerli sayilir.
  if (!accountingEnabled) {
    ia = 0.0f;
    ib = 0.0f;
    ic = 0.0f;
  }

  // Faz sayisi ve guc hesabi burada cikartilir.
  uint32_t nowMs = millis();
  float iMax = ia;
  if (ib > iMax) iMax = ib;
  if (ic > iMax) iMax = ic;

  const float phaseOnThresholdA = 0.90f;
  bool anyPhase = accountingEnabled && (iMax > phaseOnThresholdA);
  bool threePhase = accountingEnabled && ((ib > phaseOnThresholdA) || (ic > phaseOnThresholdA));
  float vLineLine = 400.0f;
  float vPhase = vLineLine / 1.732f;
  float powerW = 0.0f;
  // Seans baslatma / bitirme ve gecmise yazma mantigi burada yurur.
  if (anyPhase) {
    powerW = threePhase ? (vPhase * (ia + ib + ic)) : (vPhase * iMax);
  }
  float limitA = duty_to_amps((pwmDutyPercent > 0) ? pwmDutyPercent : PWM_DUTY_32A);
  if (limitA <= 0.0f) limitA = duty_to_amps(PWM_DUTY_32A);
  g_currentLimitA = limitA;

  if (anyPhase) {
    if (!g_sessionLive) {
      g_sessionLive = true;
      sessionStartMs = nowMs;
      sessionStartEnergyWh = energyWh;
      sessionPhaseMax = threePhase ? 3 : 1;
      g_sessionLiveStartSec = sessionStartMs / 1000;
    } else if (threePhase) {
      sessionPhaseMax = 3;
    }

    if (chargeSegmentStartMs == 0) {
      chargeSegmentStartMs = nowMs;
      lastEnergyMs = nowMs;
    }
    uint32_t dtMs = nowMs - lastEnergyMs;
    energyWh += (powerW * (dtMs / 3600000.0f));
    lastEnergyMs = nowMs;

    g_sessionLiveSeconds = (nowMs - sessionStartMs) / 1000;
    float liveWh = energyWh - sessionStartEnergyWh;
    if (liveWh < 0.0f) liveWh = 0.0f;
    g_sessionLiveEnergyKWh = liveWh / 1000.0f;
  } else {
    if (g_sessionLive) {
      uint32_t durSec = (nowMs - sessionStartMs) / 1000;
      float deltaWh = energyWh - sessionStartEnergyWh;
      if (deltaWh < 0.0f) deltaWh = 0.0f;
      float avgW = (durSec > 0) ? (deltaWh * 3600.0f / durSec) : 0.0f;

      int idx = g_histHead;
      g_histStartSec[idx] = sessionStartMs / 1000;
      g_histDurationSec[idx] = durSec;
      g_histEnergyKWh[idx] = deltaWh / 1000.0f;
      g_histAvgPowerW[idx] = avgW;
      g_histPhaseCount[idx] = sessionPhaseMax;
      g_histHead = (g_histHead + 1) % 20;
      if (g_histCount < 20) g_histCount++;

      g_sessionLive = false;
      g_sessionLiveSeconds = 0;
      g_sessionLiveEnergyKWh = 0.0f;
      g_sessionLiveStartSec = 0;
    }

    if (chargeSegmentStartMs != 0) {
      chargeAccumMs += (nowMs - chargeSegmentStartMs);
      chargeSegmentStartMs = 0;
    }
    lastEnergyMs = 0;
  }

  uint32_t chargeMs = chargeAccumMs;
  if (chargeSegmentStartMs != 0) {
    chargeMs += (nowMs - chargeSegmentStartMs);
  }
  uint32_t chargeSeconds = chargeMs / 1000;
  float energyKWh = energyWh / 1000.0f;

  g_powerW = powerW;
  g_energyKWh = energyKWh;
  g_chargeSeconds = chargeSeconds;
  g_phaseCount = threePhase ? 3 : 1;

  // OLED ekrani burada sadece okunabilir son verilerle beslenir.
  oled_draw(m.stateStable, ia, ib, ic, powerW, energyKWh, chargeSeconds, relaySet, staOk, cableConnected);
  // LED map:
  // - STATE_LED_PIN (GPIO8): C'de normal blink, D'de cift flash
  // - WIFI_LED_PIN  (GPIO17): Wi-Fi bagliyken ON
  // - ERROR_LED_PIN (GPIO18): state E/F iken ON
  static uint32_t ledTickMs = 0;
  static bool ledPhase = false;
  const uint32_t ledBlinkMs = 200;
  uint32_t ledNowMs = millis();
  if (ledNowMs - ledTickMs >= ledBlinkMs) {
    ledTickMs = ledNowMs;
    ledPhase = !ledPhase;
  }
  bool stateBlink = false;
  if (m.stateStable == "C") {
    stateBlink = ledPhase;
  } else if (m.stateStable == "D") {
    const uint32_t dCycleMs = 1200;
    const uint32_t dPhaseMs = ledNowMs % dCycleMs;
    stateBlink = (dPhaseMs < 120) || (dPhaseMs >= 240 && dPhaseMs < 360);
  }
  bool manualStopAlertOn = (g_manualStopAlertUntilMs != 0 && ((int32_t)(g_manualStopAlertUntilMs - ledNowMs) > 0));
  bool errorOn = (m.stateStable == "E" || m.stateStable == "F" || manualStopAlertOn);
  digitalWrite(STATE_LED_PIN, stateBlink ? HIGH : LOW);
  digitalWrite(WIFI_LED_PIN, staOk ? HIGH : LOW);
  digitalWrite(ERROR_LED_PIN, errorOn ? HIGH : LOW);

  // Manuel stop 60 saniye sonra otomatik AUTO moduna donsün.
  if (g_chargeMode == 2 &&
      g_manualStopAutoResumeAtMs != 0 &&
      (int32_t)(ledNowMs - g_manualStopAutoResumeAtMs) >= 0) {
    g_chargeMode = 0;
    g_manualStopAutoResumeAtMs = 0;
  }



  // IEC state'e gore hedef PWM durumu burada belirlenir.
  bool nextPwmEnabled = false;
  int nextDuty = 0;

  if (m.stateStable == "A" || m.stateStable == "E" || m.stateStable == "F") {
    // Araç yok veya hata: PWM Kapalı (Sabit 12V)
    nextPwmEnabled = false;
    nextDuty = 0;
  }
  else if (m.stateStable == "B") {
    // Araç bağlandı: PWM başlat
    nextPwmEnabled = true;
    nextDuty = PWM_DUTY_32A;
  }
  else if (m.stateStable == "C" || m.stateStable == "D") {
    // Şarjda: PWM devam
    nextPwmEnabled = true;
    nextDuty = PWM_DUTY_32A;
  }

  // Web arayuzundeki manuel START / STOP istegi burada ana kararin ustune yazilir.
  if (g_chargeMode == 2) { // STOP
    nextPwmEnabled = false;
    nextDuty = 0;
  } else if (g_chargeMode == 1) { // START
    if (m.stateStable == "B" || m.stateStable == "C" || m.stateStable == "D") {
      nextPwmEnabled = true;
      nextDuty = PWM_DUTY_32A;
    }
  }

  // Sadece değişiklik varsa uygula
  if (nextPwmEnabled != pwmEnabled || nextDuty != pwmDutyPercent) {
    pwmEnabled = nextPwmEnabled;
    pwmDutyPercent = nextDuty;
    pilot_apply_pwm();

    Serial.printf("DURUM DEĞİŞTİ: State=%s, PWM=%s, Duty=%%%d\n",
                  m.stateStable.c_str(),
                  pwmEnabled ? "ON" : "OFF",
                  pwmDutyPercent);
  }

  // Manuel stop modunda latch pulse takibini durdur.
  if (g_chargeMode != 2) {
    relay_handle_state_pulse(m.stateStable);
  }

  // Son role karari burada verilir.
  relay_update_auto(m.stateStable, pwmEnabled, pwmDutyPercent);
}
