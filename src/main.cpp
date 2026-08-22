#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <esp_system.h>

#include "app_config.h"
#include "app_pins.h"
#include "pilot/pilot.h"
#include "io/relay.h"
#include "net/web_ui.h"
#include "ui/oled_ui.h"
#include "io/current_sensor.h"
#include "OTA_Manager.h"
#include "telegram_notify.h"

// Kart kimligi (MAC tabanli)
String g_boardId = "KART-1";
String g_boardMac = "";
String g_boardName = "Eski Kart";
String g_boardCustomName = "";

// Yeni baglanan kartlara otomatik verilen Turkce endemik bitki isimleri.
// (Turkiye'nin endemik flora hazinesinden; Helium tarzi IoT isimlendirme)
// Isimler Ingilizce karakterlerle yazilir (Telegram komut uyumlulugu icin).
// Ayni MAC her zaman ayni ismi alir (deterministik secim).
static const char* const kRareNames[] = {
  "Ters Lale", "Toros Sediri", "Datca Hurmasi", "Isparta Gulu", "Anzer Cicegi",
  "Safran", "Ebe Gumusu", "Kackar Karanfili", "Salep", "Kaz Dagi Goknari",
  "Anadolu Kestanesi", "Munzur Nergisi", "Amanos Meneksesi", "Beydagi Kekigi", "Uludag Adacayi",
  "Nemrut Gelincigi", "Tunceli Kekigi", "Anadolu Findigi", "Girit Hurmasi", "Aladag Suseni"
};
static constexpr size_t kRareNameCount = sizeof(kRareNames) / sizeof(kRareNames[0]);

static void initBoardIdentity()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  g_boardMac = String(macStr);

  if (g_boardMac == "3C:DC:75:55:3B:48") {
    g_boardId = "KART-2";
    g_boardName = "Toros Sediri";
  } else if (g_boardMac == "3C:DC:75:55:3A:D0") {
    g_boardId = "KART-1";
    g_boardName = "Ters Lale";
  } else {
    g_boardId = "KART-?";
    // Bilinen listede olmayan yeni kart: MAC'ten deterministik endemik bitki ismi.
    g_boardName = kRareNames[mac[5] % kRareNameCount];
  }

  Serial.printf("[BOARD] ID=%s MAC=%s NAME=%s\n",
                g_boardId.c_str(), g_boardMac.c_str(), g_boardName.c_str());
}

static constexpr char kOtaManifestUrl[] =
  "https://raw.githubusercontent.com/turgaycam/evseyedek/backup-new-board-ota/version.json";
static constexpr char kGitHubFingerprint[] = "";
static constexpr uint32_t kOtaAutoCheckIntervalMs = 10UL * 60UL * 1000UL;
static constexpr uint8_t kMaxConsecutiveWdtResets = 3;
static constexpr uint8_t kQuickResetFactoryThreshold = 5;
static constexpr uint32_t kQuickResetClearAfterMs = 30000;
static constexpr int kForceFactoryPin = 0;             // BOOT butonu (GPIO0)
static constexpr uint32_t kForceFactoryHoldMs = 5000;  // 5 saniye
static bool sFactoryRestartPending = false;
static bool sFactoryButtonPressActive = false;
static bool sFactoryButtonHoldHandled = false;
static uint32_t sFactoryButtonPressStartMs = 0;
static uint8_t sQuickResetBootStreak = 0;
static bool sQuickResetClearArmed = false;
static uint32_t sQuickResetClearStartMs = 0;
static esp_reset_reason_t sLastResetReason = ESP_RST_UNKNOWN;

static bool isWdtResetReason(esp_reset_reason_t reason)
{
  return reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT;
}

static bool isQuickResetReason(esp_reset_reason_t reason)
{
  return reason == ESP_RST_POWERON || reason == ESP_RST_EXT || reason == ESP_RST_BROWNOUT;
}

static bool switchBootToFactory()
{
  const esp_partition_t* factory = esp_partition_find_first(
    ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr
  );
  if (factory == nullptr) {
    Serial.println("[BOOTCTL] Factory partition bulunamadi");
    return false;
  }
  esp_err_t err = esp_ota_set_boot_partition(factory);
  if (err != ESP_OK) {
    Serial.printf("[BOOTCTL] Factory secilemedi: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

static void handleForceFactoryByButton()
{
  pinMode(kForceFactoryPin, INPUT_PULLUP);
  uint32_t startMs = millis();
  while (millis() - startMs < kForceFactoryHoldMs) {
    if (digitalRead(kForceFactoryPin) != LOW) {
      return;
    }
    delay(10);
  }

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
    Serial.println("[BOOTCTL] Zaten factory partitionda");
    return;
  }

  if (switchBootToFactory()) {
    Serial.println("[BOOTCTL] GPIO0 ile factory secildi; buton birakilinca yeniden baslatilacak");
    while (digitalRead(kForceFactoryPin) == LOW) {
      delay(10);
    }
    delay(100);
    esp_restart();
  }
}

static void pollForceFactoryByButtonRuntime()
{
  bool pressed = (digitalRead(kForceFactoryPin) == LOW);
  if (!pressed) {
    sFactoryButtonPressActive = false;
    sFactoryButtonHoldHandled = false;
    sFactoryButtonPressStartMs = 0;
    return;
  }

  if (!sFactoryButtonPressActive) {
    sFactoryButtonPressActive = true;
    sFactoryButtonHoldHandled = false;
    sFactoryButtonPressStartMs = millis();
    return;
  }

  if (sFactoryButtonHoldHandled || (millis() - sFactoryButtonPressStartMs) < kForceFactoryHoldMs) return;
  sFactoryButtonHoldHandled = true;

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
    Serial.println("[BOOTCTL] Runtime hold algilandi ama zaten factory partitionda");
    return;
  }

  if (switchBootToFactory()) {
    Serial.println("[BOOTCTL] Runtime hold ile factory secildi; buton birakilinca yeniden baslatilacak");
    sFactoryRestartPending = true;
  }
}

static void processPendingFactoryRestart()
{
  if (!sFactoryRestartPending) return;
  if (digitalRead(kForceFactoryPin) == LOW) return;

  sFactoryRestartPending = false;
  Serial.println("[BOOTCTL] Buton birakildi, factory icin yeniden baslatiliyor");
  delay(100);
  esp_restart();
}

static void ensureForceFactoryPinMode()
{
  static bool configured = false;
  if (configured) return;
  pinMode(kForceFactoryPin, INPUT_PULLUP);
  configured = true;
}

int factoryButtonPin()
{
  return kForceFactoryPin;
}

bool factoryButtonIsPressed()
{
  ensureForceFactoryPinMode();
  return digitalRead(kForceFactoryPin) == LOW;
}

uint32_t factoryButtonHoldMs()
{
  if (!sFactoryButtonPressActive || sFactoryButtonPressStartMs == 0) return 0;
  return millis() - sFactoryButtonPressStartMs;
}

bool factoryButtonRestartPending()
{
  return sFactoryRestartPending;
}

uint8_t factoryQuickResetStreak()
{
  return sQuickResetBootStreak;
}

bool factoryQuickResetClearArmed()
{
  return sQuickResetClearArmed;
}

uint32_t factoryQuickResetClearRemainingMs()
{
  if (!sQuickResetClearArmed) return 0;
  uint32_t elapsed = millis() - sQuickResetClearStartMs;
  if (elapsed >= kQuickResetClearAfterMs) return 0;
  return kQuickResetClearAfterMs - elapsed;
}

int factoryLastResetReason()
{
  return (int)sLastResetReason;
}

static void serviceFactoryButton()
{
  ensureForceFactoryPinMode();
  processPendingFactoryRestart();
  if (sFactoryRestartPending) return;
  pollForceFactoryByButtonRuntime();
}

static void initFactoryButton()
{
  ensureForceFactoryPinMode();
  handleForceFactoryByButton();
  if (sFactoryRestartPending) {
    processPendingFactoryRestart();
  }
}

static void handleRescueFallbackIfNeeded()
{
  Preferences prefs;
  if (!prefs.begin("bootctl", false)) return;

  sLastResetReason = esp_reset_reason();
  uint8_t failCount = prefs.getUChar("wdt_cnt", 0);
  failCount = isWdtResetReason(sLastResetReason) ? (uint8_t)(failCount + 1) : 0;
  prefs.putUChar("wdt_cnt", failCount);

  const esp_partition_t* running = esp_ota_get_running_partition();
  bool runningFactory =
    (running != nullptr && running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY);
  uint8_t quickResetStreak = prefs.getUChar("rst_streak", 0);
  if (runningFactory) {
    quickResetStreak = 0;
  } else if (isQuickResetReason(sLastResetReason) && quickResetStreak < 255) {
    quickResetStreak++;
  }
  prefs.putUChar("rst_streak", quickResetStreak);
  sQuickResetBootStreak = quickResetStreak;
  sQuickResetClearArmed = (!runningFactory && quickResetStreak > 0);
  sQuickResetClearStartMs = millis();

  if (failCount >= kMaxConsecutiveWdtResets && switchBootToFactory()) {
    prefs.putUChar("wdt_cnt", 0);
    prefs.end();
    Serial.println("[BOOTCTL] 3x WDT reset algilandi, factory secildi");
    delay(50);
    esp_restart();
  }

  if (!runningFactory &&
      quickResetStreak >= kQuickResetFactoryThreshold &&
      switchBootToFactory()) {
    prefs.putUChar("rst_streak", 0);
    prefs.end();
    sQuickResetBootStreak = 0;
    sQuickResetClearArmed = false;
    Serial.printf("[BOOTCTL] %u hizli reset algilandi, factory secildi\n", quickResetStreak);
    delay(50);
    esp_restart();
  }

  prefs.end();
}

static void clearQuickResetStreakIfStable()
{
  if (!sQuickResetClearArmed) return;
  if ((millis() - sQuickResetClearStartMs) < kQuickResetClearAfterMs) return;

  Preferences prefs;
  if (!prefs.begin("bootctl", false)) return;
  prefs.putUChar("rst_streak", 0);
  prefs.end();

  sQuickResetBootStreak = 0;
  sQuickResetClearArmed = false;
  sQuickResetClearStartMs = 0;
  Serial.println("[BOOTCTL] Hizli reset sayaci temizlendi");
}

// Bu dosya projenin merkez akisidir.
// Neyi nereden degistirecegini hizli bulmak icin:
// - pin degisimi: include/app_pins.h
// - CP / state esikleri: bu dosyadaki global ayarlar
// - pilot state mantigi: src/pilot/pilot.cpp
// - role gecikmesi: src/io/relay.cpp
// - OLED gorunumu: src/ui/oled_ui.cpp
// - web panel ve API: src/net/web_ui.cpp

// CP yorumlama ayarlari. Web panelinden de guncellenebilir.
float CP_DIVIDER_RATIO = 4.62f;
float TH_A_MIN = 11.0f;
float TH_B_MIN = 9.6f;
float TH_C_MIN = 7.8f;
float TH_D_MIN = 5.0f;
float TH_E_MIN = 1.0f;
float marginUp = 0.30f;
float marginDown = 0.30f;
int   stableCount = 3;
int   loopIntervalMs = 200; // Varsayilan olcum hizi 200 ms

float g_powerW = 0.0f;
float g_energyKWh = 0.0f;
uint32_t g_chargeSeconds = 0;
int g_phaseCount = 1;
float g_currentLimitA = 32.0f;
float g_targetCurrentLimitA = 32.0f;

// Hizli kontrol modu: 0=AUTO, 1=START, 2=STOP
int g_chargeMode = 0;
uint32_t g_manualStopAlertUntilMs = 0;
uint32_t g_manualStopAutoResumeAtMs = 0;

// Aktif seans bilgisi
bool g_sessionLive = false;
uint32_t g_sessionLiveStartSec = 0;
uint32_t g_sessionLiveSeconds = 0;
float g_sessionLiveEnergyKWh = 0.0f;

// Son seanslar cihaz acik kaldigi surece RAM'de tutulur.
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
  // Anlik enerji ve aktif seans verilerini sifirlar.
  // Web tarafindaki "simdi sifirla" akisi burayi kullanir.
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

// IEC 61851 duty -> akim limiti yaklasimi.
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

static int amps_to_duty(float amps)
{
  if (amps < 6.0f) amps = 6.0f;
  if (amps > 32.0f) amps = 32.0f;
  int dutyPercent = (int)lroundf(amps / 0.6f);
  if (dutyPercent < 10) dutyPercent = 10;
  if (dutyPercent > 85) dutyPercent = 85;
  return dutyPercent;
}

void setup()
{
  // Baslatma sirasi bilincli tutuldu:
  // 1) web/OTA, 2) OLED, 3) sensorler, 4) role, 5) pilot
  Serial.begin(115200);
  delay(200);
  Serial.println("BOOT OK");
  initBoardIdentity();

  {
    Preferences prefs;
    if (prefs.begin("board", true)) {
      g_boardCustomName = prefs.getString("name", "");
      prefs.end();
      if (g_boardCustomName.length() > 0) {
        Serial.printf("[BOARD] Ozel isim: %s\n", g_boardCustomName.c_str());
      }
    }
    // Otomatik atanan endemik isim ilk boot'ta kalici olarak kaydedilir.
    // Boylece Telegram'da dogrudan isim gorunur ("Ters Lale"), KART-x degil.
    if (g_boardCustomName.length() == 0 && g_boardName.length() > 0 &&
        g_boardName != "Eski Kart") {
      if (prefs.begin("board", false)) {
        prefs.putString("name", g_boardName);
        prefs.end();
      }
      g_boardCustomName = g_boardName;
      Serial.printf("[BOARD] Otomatik isim kaydedildi: %s\n", g_boardCustomName.c_str());
    }
  }

  initFactoryButton();
  handleRescueFallbackIfNeeded();

  pinMode(STATE_LED_PIN, OUTPUT);
  pinMode(WIFI_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(PP_ADC_PIN, INPUT);
  digitalWrite(STATE_LED_PIN, LOW);
  digitalWrite(WIFI_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);


  // Web + OTA
  web_init();
  OTA_Manager::begin(kOtaManifestUrl, kOtaAutoCheckIntervalMs, kGitHubFingerprint);

  // OLED
  oled_init();

  // Akim sensoru
  current_sensor_init();

  // Role
  relay_init();
  relay_set_auto_enabled(true);
  relay_set_min_switch_ms(0);     // Yeni kartta kontaktor cikisi gecikmesiz seviye surulur.

  // Test dirençleri

  // Pilot
  pilot_init();

  // Ilk PWM durumu
  pwmDutyPercent = amps_to_duty(g_targetCurrentLimitA);
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

  serviceFactoryButton();
  clearQuickResetStreakIfStable();

  // Web sunucu dongusu
  web_loop();
  OTA_Manager::loop();

  // Akim sensoru dongusu
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

  if (staOk) {
    telegram_notify_connect(m.stateStable);
    telegram_notify_state_change(m.stateStable);
    telegram_notify_wifi_link(true);
    telegram_loop(m.stateStable);
  } else {
    telegram_notify_wifi_link(false);
  }

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
  float limitA = duty_to_amps((pwmDutyPercent > 0) ? pwmDutyPercent : amps_to_duty(g_targetCurrentLimitA));
  if (limitA <= 0.0f) limitA = g_targetCurrentLimitA;
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
  // - STATE_LED_PIN: C'de normal blink, D'de cift flash
  // - WIFI_LED_PIN: Wi-Fi bagliyken blink
  // - ERROR_LED_PIN: state E/F iken ON
  static uint32_t ledTickMs = 0;
  static bool ledPhase = false;
  static bool lastStaOkForLed = false;
  const uint32_t ledBlinkMs = staOk ? 1000 : 150;
  uint32_t ledNowMs = millis();
  if (staOk != lastStaOkForLed) {
    ledTickMs = ledNowMs;
    ledPhase = true;
    lastStaOkForLed = staOk;
  }
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
  digitalWrite(WIFI_LED_PIN, ledPhase ? HIGH : LOW);
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
    nextDuty = amps_to_duty(g_targetCurrentLimitA);
  }
  else if (m.stateStable == "C" || m.stateStable == "D") {
    // Şarjda: PWM devam
    nextPwmEnabled = true;
    nextDuty = amps_to_duty(g_targetCurrentLimitA);
  }

  // Web arayuzundeki manuel START / STOP istegi burada ana kararin ustune yazilir.
  if (g_chargeMode == 2) { // STOP
    nextPwmEnabled = false;
    nextDuty = 0;
  } else if (g_chargeMode == 1) { // START
    if (m.stateStable == "B" || m.stateStable == "C" || m.stateStable == "D") {
      nextPwmEnabled = true;
      nextDuty = amps_to_duty(g_targetCurrentLimitA);
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
