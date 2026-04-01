#include "web_ui.h"
#include <WiFi.h>
#include <WiFiMulti.h>

#include <WebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <math.h>

#include "app_config.h"
#include "app_pins.h"
#include "OTA_Manager.h"
#include "pilot/pilot.h"
#include "io/relay.h"

#include "io/current_sensor.h"

// Bu dosya 4 ana parcadan olusur:
// 1) Wi-Fi / OTA yardimcilari
// 2) Sayfaya gomulu HTML/CSS/JS
// 3) HTTP handler'lari
// 4) Route kayitlari
//
// Nereden mudahale edecegini hizli bulmak icin:
// - web ekrani gorunumu -> USER_HTML / MAIN_HTML sabitleri
// - yeni API endpoint -> yeni handleX fonksiyonu + web_init icinde server.on(...)
// - status JSON alani -> handleStatus()
// - kalibrasyon uygulama akisi -> handleCalibApply()

// main.cpp ve relay.cpp tarafindaki runtime degiskenler buradan okunur / yazilir.
extern float CP_DIVIDER_RATIO;
extern float TH_A_MIN, TH_B_MIN, TH_C_MIN, TH_D_MIN, TH_E_MIN;
extern float marginUp, marginDown;
extern int   stableCount;
extern int   loopIntervalMs;
extern uint32_t relayOnDelayMs;
extern uint32_t relayOffDelayMs;
extern float g_powerW;
extern float g_energyKWh;
extern uint32_t g_chargeSeconds;
extern int g_phaseCount;
extern float g_currentLimitA;
extern int g_chargeMode;
extern uint32_t g_manualStopAlertUntilMs;
extern uint32_t g_manualStopAutoResumeAtMs;
extern bool g_sessionLive;
extern uint32_t g_sessionLiveStartSec;
extern uint32_t g_sessionLiveSeconds;
extern float g_sessionLiveEnergyKWh;
extern uint32_t g_histStartSec[20];
extern uint32_t g_histDurationSec[20];
extern float g_histEnergyKWh[20];
extern float g_histAvgPowerW[20];
extern uint8_t g_histPhaseCount[20];
extern int g_histCount;
extern int g_histHead;
extern void resetChargeData(bool clearHistory);
extern void resetHistoryData();

static void pulseGpio(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(RELAY_LATCH_PULSE_MS);
  digitalWrite(pin, LOW);
}

#ifndef EVSE_AP_SSID
#define EVSE_AP_SSID "EVSE"
#endif

#ifndef EVSE_AP_PASSWORD
#define EVSE_AP_PASSWORD "12345678"
#endif

#ifndef EVSE_ADMIN_USER
#define EVSE_ADMIN_USER "admin"
#endif

#ifndef EVSE_ADMIN_PASSWORD
#define EVSE_ADMIN_PASSWORD "rotosis123"
#endif

#ifndef EVSE_HOSTNAME
#define EVSE_HOSTNAME "evse"
#endif

#ifndef EVSE_OTA_HOSTNAME
#define EVSE_OTA_HOSTNAME EVSE_HOSTNAME
#endif

#ifndef EVSE_OTA_PASSWORD
#define EVSE_OTA_PASSWORD ""
#endif

#ifndef EVSE_WIFI_1_LOC
#define EVSE_WIFI_1_LOC "Ev"
#endif
#ifndef EVSE_WIFI_1_SSID
#define EVSE_WIFI_1_SSID "FiberHGW_ZTN2TY"
#endif
#ifndef EVSE_WIFI_1_PASS
#define EVSE_WIFI_1_PASS "PzeuKE7X44Rs"
#endif

#ifndef EVSE_WIFI_2_LOC
#define EVSE_WIFI_2_LOC "Rotosis"
#endif
#ifndef EVSE_WIFI_2_SSID
#define EVSE_WIFI_2_SSID "Rotosis_Ofis"
#endif
#ifndef EVSE_WIFI_2_PASS
#define EVSE_WIFI_2_PASS "Rotosis2020"
#endif

#ifndef EVSE_WIFI_3_LOC
#define EVSE_WIFI_3_LOC "Ceylan Robot"
#endif
#ifndef EVSE_WIFI_3_SSID
#define EVSE_WIFI_3_SSID "CEYLAN-ROBOT"
#endif
#ifndef EVSE_WIFI_3_PASS
#define EVSE_WIFI_3_PASS "Mahfer123."
#endif

#ifndef EVSE_WIFI_4_LOC
#define EVSE_WIFI_4_LOC "Rotosis Atolye"
#endif
#ifndef EVSE_WIFI_4_SSID
#define EVSE_WIFI_4_SSID "Rotosis_Atolye"
#endif
#ifndef EVSE_WIFI_4_PASS
#define EVSE_WIFI_4_PASS "Rotosis2021@"
#endif

#ifndef EVSE_WIFI_5_LOC
#define EVSE_WIFI_5_LOC "Test"
#endif
#ifndef EVSE_WIFI_5_SSID
#define EVSE_WIFI_5_SSID "test"
#endif
#ifndef EVSE_WIFI_5_PASS
#define EVSE_WIFI_5_PASS "12345678"
#endif

static const char* kAdminUser = EVSE_ADMIN_USER;
static const char* kAdminPassword = EVSE_ADMIN_PASSWORD;
static const char* kHostName = EVSE_HOSTNAME;

struct KnownWifi {
  const char* location;
  const char* ssid;
  const char* password;
};

static const KnownWifi kKnownWifis[] = {
  {EVSE_WIFI_1_LOC, EVSE_WIFI_1_SSID, EVSE_WIFI_1_PASS},
  {EVSE_WIFI_2_LOC, EVSE_WIFI_2_SSID, EVSE_WIFI_2_PASS},
  {EVSE_WIFI_3_LOC, EVSE_WIFI_3_SSID, EVSE_WIFI_3_PASS},
  {EVSE_WIFI_4_LOC, EVSE_WIFI_4_SSID, EVSE_WIFI_4_PASS},
  {EVSE_WIFI_5_LOC, EVSE_WIFI_5_SSID, EVSE_WIFI_5_PASS}
};

WiFiMulti wifiMulti;
static WebServer server(80);
static bool wifiEventsReady = false;
static char s_jsonBuf[4096];
static bool s_serverStarted = false;
static uint32_t s_lastWebLoopMs = 0;
static uint32_t s_resetTotalCount = 0;
static uint32_t s_resetNowCount = 0;
static uint32_t s_resetHistoryCount = 0;
static uint32_t s_resetLastSec = 0;
static uint8_t s_resetLastModeId = 0;
static Preferences s_resetPrefs;
static bool s_resetPrefsReady = false;

struct ManualOtaState {
  bool active = false;
  bool updateBegun = false;
  bool success = false;
  String uploadedName;
  String lastError;
};

static ManualOtaState s_manualOta;
static bool s_manualOtaRebootPending = false;
static uint32_t s_manualOtaRebootAtMs = 0;

static bool hasText(const char* value) {
  return value != nullptr && value[0] != '\0';
}

static void resetManualOtaState() {
  s_manualOta.active = false;
  s_manualOta.updateBegun = false;
  s_manualOta.success = false;
  s_manualOta.uploadedName = "";
  s_manualOta.lastError = "";
}

static void refreshMdns() {
  static bool mdnsStarted = false;
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);

  if (!staOk) {
    if (mdnsStarted) {
      MDNS.end();
      mdnsStarted = false;
      Serial.println("[mDNS] Stopped");
    }
    return;
  }

  if (!mdnsStarted) {
    if (!MDNS.begin(kHostName)) {
      Serial.println("[mDNS] Start failed");
      return;
    }
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.print("[mDNS] Ready: http://");
    Serial.print(kHostName);
    Serial.println(".local");
  }
}

static void setupArduinoOta() {
  Serial.println("[OTA] ArduinoOTA ve generic web OTA devre disi; guvenli yukleme /update uzerinden yapilir");
}

// Web API'de gelen tamsayi parametreleri guvenli aralikta tutar.
static int clampIntArg(const String& v, int minVal, int maxVal) {
  long parsed = v.toInt();
  if (parsed < minVal) return minVal;
  if (parsed > maxVal) return maxVal;
  return (int)parsed;
}

static float clampFloatArg(const String& v, float minVal, float maxVal, float fallback) {
  float parsed = v.toFloat();
  if (!(parsed == parsed)) return fallback; // NaN guard
  if (parsed < minVal) return minVal;
  if (parsed > maxVal) return maxVal;
  return parsed;
}

static float safeFinite(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  return v;
}

static const char* wifiLocationForSsid(const String& connectedSsid) {
  for (size_t i = 0; i < (sizeof(kKnownWifis) / sizeof(kKnownWifis[0])); i++) {
    if (connectedSsid == kKnownWifis[i].ssid) return kKnownWifis[i].location;
  }
  return "Bilinmiyor";
}

static bool requireAdminAuth() {
  if (server.authenticate(kAdminUser, kAdminPassword)) {
    return true;
  }
  server.requestAuthentication(BASIC_AUTH, "RotosisEVSE Admin", "Sifre gerekli");
  return false;
}

static const char* chargeModeLabel(int mode) {
  if (mode == 1) return "START";
  if (mode == 2) return "STOP";
  return "AUTO";
}

static const char* resetModeLabel(uint8_t modeId) {
  if (modeId == 1) return "ANLIK";
  if (modeId == 2) return "GECMIS";
  if (modeId == 3) return "ANLIK+GECMIS";
  return "YOK";
}

struct ResetStatsPersist {
  uint32_t total;
  uint32_t nowCount;
  uint32_t historyCount;
  uint32_t lastSec;
  uint8_t lastModeId;
};

static void loadResetStats() {
  if (!s_resetPrefsReady) return;
  ResetStatsPersist stats = {};
  size_t got = s_resetPrefs.getBytes("rstStats", &stats, sizeof(stats));
  if (got != sizeof(stats)) return;

  s_resetTotalCount = stats.total;
  s_resetNowCount = stats.nowCount;
  s_resetHistoryCount = stats.historyCount;
  s_resetLastSec = stats.lastSec;
  s_resetLastModeId = (stats.lastModeId <= 3) ? stats.lastModeId : 0;
}

static void saveResetStats() {
  if (!s_resetPrefsReady) return;
  ResetStatsPersist stats = {};
  stats.total = s_resetTotalCount;
  stats.nowCount = s_resetNowCount;
  stats.historyCount = s_resetHistoryCount;
  stats.lastSec = s_resetLastSec;
  stats.lastModeId = s_resetLastModeId;
  s_resetPrefs.putBytes("rstStats", &stats, sizeof(stats));
}

static void noteResetEvent(bool clearNow, bool clearHistory) {
  if (!clearNow && !clearHistory) return;
  s_resetTotalCount++;
  if (clearNow) s_resetNowCount++;
  if (clearHistory) s_resetHistoryCount++;
  s_resetLastSec = millis() / 1000UL;
  if (clearNow && clearHistory) {
    s_resetLastModeId = 3;
  } else if (clearNow) {
    s_resetLastModeId = 1;
  } else {
    s_resetLastModeId = 2;
  }
  saveResetStats();
}

// 2) Sayfaya gomulu on yuz kaynaklari burada baslar.
static const char USER_HTML[] PROGMEM = R"HTML(

<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>RotosisEVSE</title>
<style>
*{box-sizing:border-box}
:root{--bg:#06121d;--bg2:#0a2233;--panel:rgba(8,24,37,.78);--panelStrong:rgba(12,31,47,.92);--line:rgba(72,209,164,.18);--text:#ecfbf6;--muted:#8daea4;--accent:#37d8a2;--accentDeep:#15b987;--accentSoft:rgba(55,216,162,.18);--warn:#b6ef72;--err:#ff7d7d;--shadow:0 24px 50px rgba(1,9,16,.46);--shadowSoft:0 14px 32px rgba(2,10,18,.34);}
html,body{margin:0;padding:0}
body{position:relative;overflow-x:hidden;font-family:"Avenir Next","Segoe UI","Trebuchet MS",sans-serif;color:var(--text);min-height:100vh;background:
radial-gradient(circle at 18% 16%, rgba(34,219,167,.18) 0%, rgba(34,219,167,0) 28%),
radial-gradient(circle at 84% 14%, rgba(65,177,255,.18) 0%, rgba(65,177,255,0) 26%),
linear-gradient(180deg,#07131f 0%,#0a1d2f 46%,#05101a 100%);}
body::before,body::after{content:"";position:fixed;z-index:0;border-radius:999px;pointer-events:none;filter:blur(18px);}
body::before{width:260px;height:260px;top:74px;left:-88px;background:radial-gradient(circle, rgba(56,223,169,.22) 0%, rgba(56,223,169,0) 72%);animation:floatGlowA 16s ease-in-out infinite;}
body::after{width:290px;height:290px;right:-106px;bottom:92px;background:radial-gradient(circle, rgba(67,157,255,.18) 0%, rgba(67,157,255,0) 72%);animation:floatGlowB 18s ease-in-out infinite;}
body.state-A{--accent:#66c7ff;--accentDeep:#249ff0;--accentSoft:rgba(102,199,255,.18)}
body.state-B{--accent:#7ae6c4;--accentDeep:#2fc79a;--accentSoft:rgba(122,230,196,.18)}
body.state-C,body.state-D{--accent:#37d8a2;--accentDeep:#15b987;--accentSoft:rgba(55,216,162,.18)}
body.state-E,body.state-F{--accent:#ff8b8b;--accentDeep:#ff6464;--accentSoft:rgba(255,139,139,.18)}
.app{position:relative;z-index:1;max-width:430px;margin:0 auto;padding:18px 18px 28px;animation:screenEnter .82s cubic-bezier(.22,1,.36,1);}
.topbar{display:grid;grid-template-columns:44px 1fr 52px;align-items:center;gap:10px;}
.backBtn{width:44px;height:44px;border-radius:16px;display:grid;place-items:center;text-decoration:none;color:#dff9f0;font-size:30px;line-height:1;background:linear-gradient(180deg,rgba(18,42,61,.92),rgba(9,22,35,.88));border:1px solid rgba(79,225,178,.16);box-shadow:var(--shadowSoft);}
.statusWrap{display:flex;justify-content:center;}
.statusPill{display:inline-flex;align-items:center;gap:10px;padding:11px 16px;border-radius:999px;background:linear-gradient(135deg,rgba(13,31,46,.96),rgba(7,20,31,.9));border:2px solid color-mix(in srgb,var(--accentDeep) 52%, white);box-shadow:0 16px 30px rgba(2,10,18,.28);font-weight:800;font-size:14px;letter-spacing:.01em;color:#eefcf8;animation:fadeLift .82s .04s both;}
.statusIcon{width:24px;height:24px;display:grid;place-items:center;color:var(--accent);}
.statusIcon svg{width:24px;height:24px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round;}
.vehicleName{font-weight:800;}
.pillDash{opacity:.55}
.sync{justify-self:end;min-width:52px;height:36px;padding:0 10px;border-radius:999px;background:linear-gradient(180deg,rgba(15,35,54,.94),rgba(8,19,30,.9));border:1px solid rgba(87,209,171,.16);display:flex;align-items:center;justify-content:center;gap:7px;font-size:11px;font-weight:800;letter-spacing:.08em;color:#8fbbb0;text-transform:uppercase;box-shadow:var(--shadowSoft);}
.sync::before{content:"";width:8px;height:8px;border-radius:999px;background:#3e6f84;box-shadow:0 0 0 4px rgba(62,111,132,.18);}
.sync.live{color:#dffff4;border-color:rgba(55,216,162,.26);background:linear-gradient(180deg,rgba(18,49,58,.96),rgba(9,26,35,.94));}
.sync.live::before{background:var(--accent);box-shadow:0 0 0 4px color-mix(in srgb,var(--accent) 18%, transparent);animation:livePulse 1.8s ease-in-out infinite;}
.headline{padding:14px 8px 0;text-align:center;animation:fadeLift .88s .1s both;}
.dateLine{font-size:18px;font-weight:700;letter-spacing:.01em;color:#bde9ff;}
.locationLine{margin-top:10px;font-size:18px;line-height:1.35;font-weight:700;color:#f1fffb;}
.hintLine{margin-top:6px;font-size:13px;color:#84c8b5;}
.carStage{margin-top:20px;padding:16px 12px 10px;border-radius:34px;background:linear-gradient(180deg,rgba(10,28,42,.72),rgba(6,16,26,.34));border:1px solid rgba(87,215,175,.14);box-shadow:var(--shadowSoft);animation:fadeLift .94s .16s both;}
.carWrap{position:relative;max-width:330px;min-height:198px;margin:0 auto;display:flex;align-items:center;justify-content:center;isolation:isolate;}
.carWrap::before{content:"";position:absolute;inset:13% 5% 15%;border-radius:34px;background:linear-gradient(180deg,rgba(18,43,62,.72),rgba(7,20,31,.16));border:1px solid rgba(172,244,221,.08);box-shadow:inset 0 1px 0 rgba(255,255,255,.06);backdrop-filter:blur(10px);z-index:0;}
.carWrap::after{content:"";position:absolute;left:14%;right:14%;bottom:10%;height:58px;border-radius:999px;background:radial-gradient(ellipse at center, rgba(42,221,169,.18) 0%, rgba(42,221,169,.09) 30%, rgba(6,16,26,0) 74%),radial-gradient(ellipse at center, rgba(4,12,20,.5) 0%, rgba(4,12,20,.2) 42%, rgba(4,12,20,0) 76%);filter:blur(8px);z-index:1;animation:platformPulse 6.5s ease-in-out infinite;}
.carHalo{position:absolute;inset:8% 6% 12%;border-radius:48px;background:radial-gradient(circle at 50% 48%, rgba(210,255,243,.18) 0%, color-mix(in srgb,var(--accent) 48%, white) 22%, rgba(65,177,255,.16) 52%, rgba(255,255,255,0) 82%);filter:blur(14px);opacity:.94;z-index:2;animation:haloBreath 6.2s ease-in-out infinite;}
.carFill{position:absolute;inset:0;--fillColor:rgba(124,210,112,.86);--alpha:0;--car-img:none;-webkit-mask-image:var(--car-img);mask-image:var(--car-img);-webkit-mask-size:contain;mask-size:contain;-webkit-mask-repeat:no-repeat;mask-repeat:no-repeat;-webkit-mask-position:center;mask-position:center;overflow:hidden;opacity:var(--alpha);transition:opacity .7s ease;z-index:3;}
.carFillInner{position:absolute;left:0;right:0;bottom:0;height:0%;background:linear-gradient(180deg,color-mix(in srgb,var(--accent) 46%, white) 0%, var(--accent) 48%, var(--accentDeep) 100%);box-shadow:0 0 28px color-mix(in srgb,var(--accentDeep) 32%, transparent) inset;transition:height .55s ease,background-color .55s ease;}
.carFill.stateA{--alpha:0}
.carFill.stateB{--alpha:.72;--fillColor:rgba(122,230,196,.86)}
.carFill.stateC,.carFill.stateD{--alpha:1;--fillColor:rgba(55,216,162,.92)}
.carFill.stateC .carFillInner,.carFill.stateD .carFillInner{animation:carChargeFill 5.8s ease-in-out infinite;}
.carFill.stateE,.carFill.stateF{--alpha:1;--fillColor:rgba(255,125,125,.92)}
.carFill.stateE .carFillInner,.carFill.stateF .carFillInner{animation:carFaultPulse 1.1s ease-in-out infinite;}
.carSvg{position:relative;z-index:4;width:100%;max-width:330px;height:auto;display:block;transform:translateY(-2px);filter:drop-shadow(0 26px 26px rgba(1,9,16,.42));animation:carFloat 7s ease-in-out infinite;}
.batteryRow{margin-top:16px;display:flex;align-items:center;gap:10px;padding:14px 16px;border-radius:24px;background:linear-gradient(180deg,rgba(12,31,46,.9),rgba(6,17,28,.82));border:1px solid rgba(76,210,166,.14);box-shadow:var(--shadowSoft);font-size:22px;font-weight:700;letter-spacing:-.01em;animation:fadeLift .96s .22s both;}
.batteryText{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}
.batteryRow span:last-child{margin-left:auto;flex:0 0 auto;}
.batteryIcon{width:28px;height:28px;display:grid;place-items:center;color:#4de2b0;}
.batteryIcon svg{width:28px;height:28px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round;}
.metricCard{margin-top:18px;padding:20px 22px;border-radius:26px;background:linear-gradient(180deg,var(--panelStrong),var(--panel));border:1px solid var(--line);box-shadow:var(--shadow);backdrop-filter:blur(12px);animation:fadeLift 1s .26s both;}
.metricLabel{font-size:16px;color:#91cbb9;font-weight:700;}
.metricHero{margin-top:10px;font-size:44px;line-height:1;font-weight:800;letter-spacing:-.04em;color:#f1fffb;}
.metricHero .unit{font-size:.58em;font-weight:700;letter-spacing:-.01em;color:#7fe5c1;}
.metricsGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:22px 18px;}
.metricCell{min-width:0;}
.metricHead{display:flex;align-items:center;gap:8px;color:#9fcfc2;font-size:16px;font-weight:700;}
.metricHead svg{width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:1.9;stroke-linecap:round;stroke-linejoin:round;flex:0 0 auto;}
.metricValue{margin-top:9px;font-size:20px;font-weight:800;letter-spacing:-.03em;color:#eafcf6;}
.metricValue .unit{font-size:.82em;color:#6fd8b1;}
.phaseMeta{margin-top:16px;padding-top:14px;border-top:1px solid rgba(76,210,166,.14);font-size:13px;color:#8bb5a6;display:flex;justify-content:space-between;gap:10px;}
.footerStatus{margin-top:18px;padding:16px 18px 18px;border-radius:24px;background:linear-gradient(180deg,rgba(12,31,46,.88),rgba(6,17,28,.78));border:1px solid rgba(76,210,166,.14);box-shadow:var(--shadowSoft);animation:fadeLift 1.04s .32s both;}
.statusRow{display:flex;justify-content:space-between;align-items:center;gap:12px;}
.statusBadge{display:inline-flex;align-items:center;gap:8px;font-size:13px;font-weight:800;color:#dffcf3;}
.statusOrb{width:10px;height:10px;border-radius:999px;background:#37d8a2;box-shadow:0 0 0 5px rgba(55,216,162,.14);}
.statusOrb.warn{background:#b6ef72;box-shadow:0 0 0 5px rgba(182,239,114,.16);}
.statusOrb.err{background:#ff7d7d;box-shadow:0 0 0 5px rgba(255,125,125,.16);}
.statusMeta{font-size:12px;color:#8bb5a6;text-align:right;}
.statusText{margin-top:10px;font-size:18px;font-weight:800;line-height:1.3;color:#eefcf8;}
.statusFoot{margin-top:12px;display:grid;gap:6px;font-size:12px;color:#8bb5a6;}
.debugStrip{display:none;}
.installBtn{display:none;margin-top:14px;width:100%;padding:14px 16px;border-radius:18px;border:1px solid rgba(76,210,166,.16);background:linear-gradient(135deg,rgba(13,40,53,.96),rgba(7,22,31,.9));color:#dcfff2;font-size:14px;font-weight:800;box-shadow:var(--shadowSoft);}
@keyframes screenEnter{0%{opacity:0;transform:translateY(18px)}100%{opacity:1;transform:translateY(0)}}
@keyframes fadeLift{0%{opacity:0;transform:translateY(14px)}100%{opacity:1;transform:translateY(0)}}
@keyframes floatGlowA{0%,100%{transform:translate3d(0,0,0) scale(1)}50%{transform:translate3d(22px,18px,0) scale(1.08)}}
@keyframes floatGlowB{0%,100%{transform:translate3d(0,0,0) scale(1)}50%{transform:translate3d(-18px,-14px,0) scale(1.1)}}
@keyframes haloBreath{0%,100%{opacity:.78;transform:scale(.98)}50%{opacity:1;transform:scale(1.03)}}
@keyframes carFloat{0%,100%{transform:translateY(-2px)}50%{transform:translateY(-7px)}}
@keyframes platformPulse{0%,100%{opacity:.78;transform:scaleX(.96)}50%{opacity:1;transform:scaleX(1.02)}}
@keyframes livePulse{0%,100%{transform:scale(1)}50%{transform:scale(1.22)}}
@keyframes carChargeFill{0%{transform:translateY(0)}50%{transform:translateY(-11px)}100%{transform:translateY(0)}}
@keyframes carFaultPulse{0%,100%{filter:brightness(1) saturate(1)}50%{filter:brightness(1.16) saturate(1.18)}}
@media(max-width:390px){.app{padding:16px 14px 24px}.statusPill{padding:10px 13px;font-size:13px}.dateLine{font-size:17px}.locationLine{font-size:16px}.batteryRow{font-size:19px}.metricHero{font-size:38px}.metricValue{font-size:18px}}
</style>
<link rel="manifest" href="/manifest.json">
<meta name="theme-color" content="#0a1d2f">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="RotosisEVSE">
</head><body class="state-A">
<div class="app">
  <div class="topbar">
    <a class="backBtn" href="/settings" aria-label="Ayarlar" title="Ayarlar">&#9881;</a>
    <div class="statusWrap">
      <div class="statusPill" id="statusPill">
        <span class="statusIcon" aria-hidden="true">
          <svg viewBox="0 0 24 24"><path d="M5 14.5V9.8l2.2-2.3h9.6L19 9.8v4.7"></path><path d="M7.7 7.5l1.4-2h5.8l1.4 2"></path><path d="M8 14.5h8"></path><path d="M10.5 12l-1.2 2.5h2.1l-1 2.5 3-4h-2.1l1.1-2z"></path></svg>
        </span>
        <span id="stateName">Haz&#305;r</span>
      </div>
    </div>
    <div class="sync" id="sync">WAIT</div>
  </div>

  <div class="headline">
    <div class="dateLine" id="dateLabel">-</div>
    <div class="locationLine" id="stationLabel">EVSE &#304;stasyonu - DC</div>
    <div class="hintLine" id="stateHint">Ara&#231; bekleniyor</div>
  </div>

  <section class="carStage">
    <div class="carWrap">
      <div class="carHalo"></div>
      <div class="carFill" id="carFill"><div class="carFillInner" id="carFillInner"></div></div>
      <img id="carSvg" class="carSvg" alt="Car" src="data:image/png;base64,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" />
    </div>
  </section>

  <div class="batteryRow">
    <span class="batteryIcon" aria-hidden="true">
      <svg viewBox="0 0 24 24"><path d="M8 5h5l2 2h3v10H6V7h2z"></path><path d="M9 10h3"></path><path d="M10.5 8.5v3"></path><path d="M7 20h10"></path></svg>
    </span>
    <span class="batteryText">Batarya Seviyesi</span>
    <span id="battPct">% --</span>
  </div>

  <section class="metricCard">
    <div class="metricLabel">Anl&#305;k Aktar&#305;lan G&#252;&#231;</div>
    <div class="metricHero"><span id="pwr">0.00</span> <span class="unit">kW</span></div>
  </section>

  <section class="metricCard">
    <div class="metricsGrid">
      <div class="metricCell">
        <div class="metricHead">
          <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"></circle><path d="M12 8v4l2.5 2.5"></path></svg>
          <span>Ge&#231;en S&#252;re</span>
        </div>
        <div class="metricValue" id="tsec">0.00 <span class="unit">dk</span></div>
      </div>
      <div class="metricCell">
        <div class="metricHead">
          <svg viewBox="0 0 24 24"><path d="M12 3l1.7 4.4L18 9l-4.3 1.6L12 15l-1.7-4.4L6 9l4.3-1.6z"></path></svg>
          <span>Aktar&#305;lan Enerji</span>
        </div>
        <div class="metricValue"><span id="ekwh">0.0</span> <span class="unit">kWh</span></div>
      </div>
      <div class="metricCell">
        <div class="metricHead">
          <svg viewBox="0 0 24 24"><path d="M4 13h4l2-6 4 12 2-6h4"></path></svg>
          <span>Akim Limiti</span>
        </div>
        <div class="metricValue"><span id="limitMetric">32.0</span> <span class="unit">A</span></div>
      </div>
      <div class="metricCell">
        <div class="metricHead">
          <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"></circle><path d="M8.5 12h7"></path><path d="M12 8.5v7"></path><path d="M17.5 12h2"></path><path d="M4.5 12h2"></path></svg>
          <span>Ak&#305;m</span>
        </div>
        <div class="metricValue"><span id="currentMetric">0.0</span> <span class="unit">A</span></div>
      </div>
    </div>
    <div class="phaseMeta">
      <span id="phaseSummary">1 faz</span>
      <span id="limitMeta">32.0 A limit</span>
    </div>
  </section>

  <section class="footerStatus" id="alarmBox">
    <div class="statusRow">
      <div class="statusBadge">
        <span class="statusOrb" id="statusOrb"></span>
        <span id="statusCode">Durum A</span>
      </div>
      <div class="statusMeta" id="alarmMeta">Takip devam ediyor</div>
    </div>
    <div class="statusText" id="alarmText">Sistem normal</div>
    <div class="statusFoot">
      <div id="wifiLine">Wi-Fi: -</div>
      <div><span id="host">-</span> | <span id="ip">-</span></div>
      <div id="ts">-</div>
    </div>
    <div class="debugStrip" aria-hidden="true">
      <span id="state">STATE:A</span>
      <span id="relay">R:RESET</span>
      <span id="i1">0.0 A</span>
      <span id="i2">0.0 A</span>
      <span id="i3">0.0 A</span>
      <span id="it">0.0</span>
      <span id="limitA">32.0</span>
      <span id="loadPct">0%</span>
      <span id="loadPctCard">0%</span>
      <span id="gaugeLabel">1 faz / 32.0 A</span>
      <span id="gaugePct">0%</span>
      <span id="energyMeta">Aktif seans</span>
      <span id="timeMeta">1 faz</span>
    </div>
  </section>

  <button class="installBtn" id="installBtn">Uygulamaya Ekle</button>
</div>
<script>
let deferredPrompt=null;
const POLL_MS=1200;
const MAX_POINTS=40;
let livePoints=[];
let chartCeil=32;
let currentChargeModeId=0;
const stateMeta={
  A:{name:"Hazır",hint:"Araç bekleniyor"},
  B:{name:"Bağlı",hint:"Araç bağlandı"},
  C:{name:"Şarj Ediliyor",hint:"Enerji aktarımı sürüyor"},
  D:{name:"Havalandırma",hint:"Şarj sürüyor"},
  E:{name:"Şarj Hatası",hint:"Pilot hata durumu"},
  F:{name:"Kritik Hata",hint:"Koruma aktif"}
};
const gaugeRing=document.getElementById("gaugeValue");
const gaugeCirc=gaugeRing?(2*Math.PI*gaugeRing.r.baseVal.value):0;
if(gaugeRing){
  gaugeRing.style.strokeDasharray=gaugeCirc.toFixed(1);
  gaugeRing.style.strokeDashoffset=gaugeCirc.toFixed(1);
}
const carEls={
  carSvg:document.getElementById("carSvg"),
  carFill:document.getElementById("carFill"),
  carFillInner:document.getElementById("carFillInner"),
  battPct:document.getElementById("battPct"),
  chargeBar:document.getElementById("chargeBar"),
  chargeBarInner:document.getElementById("chargeBarInner")
};
const BAT_KWH=80;
const CAR_IMG_SRC=(carEls.carSvg&&carEls.carSvg.getAttribute("src"))?carEls.carSvg.getAttribute("src"):"";
if(CAR_IMG_SRC&&carEls.carFill){
  carEls.carFill.style.setProperty('--car-img','url('+CAR_IMG_SRC+')');
}
function updateCar(d){
  if(!carEls.carFill||!carEls.carFillInner||!carEls.battPct) return;
  const st=(d.state||"-");
  const charging=(st==="C"||st==="D");
  const isCable=(st==="B");
  const isError=(st==="E"||st==="F");
  carEls.carFill.classList.remove("stateA","stateB","stateC","stateD","stateE","stateF");
  if(st==="A") carEls.carFill.classList.add("stateA");
  else if(st==="B") carEls.carFill.classList.add("stateB");
  else if(st==="C") carEls.carFill.classList.add("stateC");
  else if(st==="D") carEls.carFill.classList.add("stateD");
  else if(st==="E") carEls.carFill.classList.add("stateE");
  else if(st==="F") carEls.carFill.classList.add("stateF");
  const socVal=Number.isFinite(d.soc)?d.soc:(Number.isFinite(d.batt)?d.batt:null);
  let pctRaw=Number.isFinite(socVal)?socVal:Math.round((Number(d.eKWh)||0)/BAT_KWH*100);
  pctRaw=Math.max(0,Math.min(100,pctRaw));
  let fillPct=pctRaw;
  if(charging) fillPct=Math.max(fillPct,8);
  else if(isCable) fillPct=Math.max(fillPct,12);
  else if(isError) fillPct=Math.max(fillPct,24);
  if(st==="A") fillPct=0;
  carEls.carFillInner.style.height=Math.max(0,Math.min(100,fillPct))+"%";
  carEls.battPct.textContent="% "+(Number.isFinite(fillPct)?Math.round(fillPct):"--");
}
function fmtTime(sec){
  const totalSec=Math.max(0,Number(sec)||0);
  const mins=totalSec/60;
  return mins.toFixed(2)+" dk";
}
function clamp(v,min,max){return Math.max(min,Math.min(max,v))}
function setText(id,value){
  const el=document.getElementById(id);
  if(el) el.textContent=value;
}
function chargeCmd(mode){fetch('/charge_cmd?m='+mode).then(()=>pull()).catch(()=>{});}
function updateChargeAction(modeId){
  currentChargeModeId=modeId;
}
function toggleChargeAction(){
  chargeCmd(currentChargeModeId===2?0:2);
}
function setSync(ok){
  const el=document.getElementById("sync");
  if(!el) return;
  el.textContent=ok?"LIVE":"WAIT";
  el.className=ok?"sync live":"sync";
}
function setGauge(loadPct){
  const pct=clamp(loadPct,0,100);
  if(gaugeRing){
    gaugeRing.style.strokeDashoffset=(gaugeCirc*(1-pct/100)).toFixed(1);
  }
  setText("gaugePct",pct.toFixed(0)+"%");
  setText("loadPct",pct.toFixed(0)+"%");
  setText("loadPctCard",pct.toFixed(0)+"%");
}
function pushLivePoint(value){
  livePoints.push(clamp(value,0,999));
  if(livePoints.length>MAX_POINTS) livePoints.shift();
  renderLiveChart();
}
function renderLiveChart(){
  const line=document.getElementById("chartLine");
  const area=document.getElementById("chartArea");
  const dot=document.getElementById("chartDot");
  if(!line||!area||!dot) return;
  const values=livePoints.length?livePoints:[0];
  const w=320,h=160,padX=12,padY=14;
  const maxVal=Math.max(chartCeil,6,...values);
  const step=values.length>1?((w-(padX*2))/(values.length-1)):0;
  const coords=values.map((v,i)=>{
    const x=padX+(step*i);
    const y=h-padY-((v/maxVal)*(h-(padY*2)));
    return [x,y];
  });
  const points=coords.map(([x,y])=>x.toFixed(1)+","+y.toFixed(1)).join(" ");
  line.setAttribute("points",points||("12,"+(h-padY)));
  const last=coords[coords.length-1]||[w-padX,h-padY];
  const areaPath="M "+padX+" "+(h-padY)+" L "+coords.map(([x,y])=>x.toFixed(1)+" "+y.toFixed(1)).join(" L ")+" L "+last[0].toFixed(1)+" "+(h-padY)+" Z";
  area.setAttribute("d",areaPath);
  dot.setAttribute("cx",last[0].toFixed(1));
  dot.setAttribute("cy",last[1].toFixed(1));
  const peak=Math.max(...values);
  const avg=values.reduce((sum,v)=>sum+v,0)/values.length;
  const windowSec=Math.max(0,Math.round(((values.length-1)*POLL_MS)/1000));
  setText("chartPeak",peak.toFixed(1)+" A");
  setText("chartAvg",avg.toFixed(1)+" A");
  setText("chartWindow",windowSec+" sn");
  setText("chartNow",values[values.length-1].toFixed(1)+" A");
}
function updateState(st){
  document.body.className="state-"+st;
  setText("state","STATE:"+st);
  const meta=stateMeta[st]||stateMeta["A"];
  setText("stateName",meta.name);
  setText("stateHint",meta.hint);
  setText("statusCode","Durum "+st);
}
function renderAlarm(level,text,state,staOk){
  const box=document.getElementById("alarmBox");
  if(box) box.className="footerStatus";
  const orb=document.getElementById("statusOrb");
  if(orb) orb.className="statusOrb"+(level===2?" err":(level===1?" warn":""));
  setText("alarmText",text||"Sistem normal");
  setText("alarmMeta","Durum kodu "+state+" | "+(staOk?"Wi-Fi bağlı":"Wi-Fi yok"));
}
function updateDateLabel(){
  setText('dateLabel',new Date().toLocaleString('tr-TR',{day:'numeric',month:'long',year:'numeric',hour:'2-digit',minute:'2-digit'}));
}
function pull(){
  fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    const ia=Number(d.ia)||0, ib=Number(d.ib)||0, ic=Number(d.ic)||0;
    const it=ia+ib+ic;
    const phaseCount=Math.max(1,Number(d.phase)||1);
    const limitA=Math.max(6,Number(d.limitA)||32);
    const totalLimit=phaseCount*limitA;
    const loadPct=clamp((it/totalLimit)*100,0,100);
    const powerKw=((Number(d.pW)||0)/1000.0);
    const liveSession=!!d.sLive;
    const energy=(liveSession&&d.sLiveKWh!==undefined)?(Number(d.sLiveKWh)||0):(Number(d.eKWh)||0);
    const timeSec=(liveSession&&d.sLiveSec!==undefined)?(Number(d.sLiveSec)||0):(Number(d.tSec)||0);
    const activePhases=Math.max(1,[ia,ib,ic].filter(v=>v>0.5).length||phaseCount);
    const displayCurrent=activePhases>1 ? (it/activePhases) : it;
    const stationBase=(d.wifiLoc&&d.wifiLoc!=="-")?d.wifiLoc:((d.host&&d.host!=="-")?d.host:"EVSE Istasyonu");
    chartCeil=Math.max(6,totalLimit);
    setText('it',displayCurrent.toFixed(1));
    setText('currentMetric',displayCurrent.toFixed(1));
    setText('i1',ia.toFixed(1)+" A");
    setText('i2',ib.toFixed(1)+" A");
    setText('i3',ic.toFixed(1)+" A");
    setText('pwr',powerKw.toFixed(2));
    setText('ekwh',energy.toFixed(1));
    setText('tsec',fmtTime(timeSec));
    setText('energyMeta',liveSession?"Aktif seans":"Son okuma");
    setText('timeMeta',phaseCount+" faz");
    setText('limitA',limitA.toFixed(1));
    setText('limitMetric',limitA.toFixed(1));
    setText('limitMeta',phaseCount+" faz / "+limitA.toFixed(1)+" A limit");
    setText('phaseSummary',phaseCount+" faz • "+loadPct.toFixed(0)+"% doluluk");
    setText('gaugeLabel',phaseCount+" faz / "+limitA.toFixed(1)+" A");
    setText('stationLabel',stationBase+" - DC");
    updateChargeAction(Number(d.modeId)||0);
    if(d.rLbl!==undefined) setText('relay',"R:"+d.rLbl);
    if(d.state!==undefined) updateState(d.state);
    if(d.ip!==undefined) setText('ip',d.ip);
    if(d.host!==undefined) setText('host',d.host);
    setGauge(loadPct);
    setText('ts',"Son güncelleme: "+new Date().toLocaleTimeString("tr-TR",{hour:"2-digit",minute:"2-digit",second:"2-digit"}));
    const wifiText=(d.wifiSsid&&d.wifiSsid!=="-") ? ("Wi-Fi: "+d.wifiSsid+((d.wifiLoc&&d.wifiLoc!=="-")?" / "+d.wifiLoc:"")) : "Wi-Fi: bağlı değil";
    setText('wifiLine',wifiText);
    renderAlarm(d.alarmLv||0, d.alarmTxt||"Sistem normal", d.state||"A", !!d.staOk);
    updateCar(d);
    pushLivePoint(displayCurrent);
    updateDateLabel();
    setSync(true);
  }).catch(()=>{ setSync(false); updateDateLabel(); });
}
window.addEventListener("beforeinstallprompt",(e)=>{
  e.preventDefault();
  deferredPrompt=e;
  const b=document.getElementById("installBtn");
  if(b) b.style.display="block";
});
document.addEventListener("click",(e)=>{
  const btn=(e.target&&e.target.closest)?e.target.closest("#installBtn"):null;
  if(btn && deferredPrompt){
    deferredPrompt.prompt();
    deferredPrompt.userChoice.finally(()=>{deferredPrompt=null;btn.style.display="none";});
  }
});
if("serviceWorker" in navigator){
  window.addEventListener("load",function(){
    navigator.serviceWorker.register("/sw.js").catch(()=>{});
  });
}
renderLiveChart();
updateDateLabel();
setInterval(pull,POLL_MS);
setInterval(updateDateLabel,30000);
pull();
</script>
</body></html>

)HTML";


static const char MANIFEST_JSON[] PROGMEM = R"JSON(
{
  "name": "RotosisEVSE",
  "short_name": "RotosisEVSE",
  "start_url": "/",
  "scope": "/",
  "display": "standalone",
  "background_color": "#08131c",
  "theme_color": "#0d1a2b",
  "icons": [
    {
      "src": "/app-icon.svg",
      "sizes": "192x192",
      "type": "image/svg+xml",
      "purpose": "any"
    },
    {
      "src": "/app-icon.svg",
      "sizes": "512x512",
      "type": "image/svg+xml",
      "purpose": "any"
    }
  ]
}
)JSON";

static const char SERVICE_WORKER_JS[] PROGMEM = R"JS(
const CACHE_NAME = "evse-pwa-v4";
const ASSETS = ["/", "/manifest.json", "/app-icon.svg"];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(ASSETS))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys.filter((k) => k !== CACHE_NAME).map((k) => caches.delete(k))
      )
    ).then(() => self.clients.claim())
  );
});

self.addEventListener("fetch", (event) => {
  const url = new URL(event.request.url);
  const path = url.pathname;
  const isLiveApi =
    path === "/status" ||
    path === "/history" ||
    path === "/data_reset" ||
    path === "/charge_cmd" ||
    path === "/calib_apply" ||
    path.startsWith("/relay") ||
    path === "/pulse_reset" ||
    path === "/pulse_set";

  if (isLiveApi) {
    event.respondWith(
      fetch(event.request).catch(() =>
        new Response("{}", { headers: { "Content-Type": "application/json" } })
      )
    );
    return;
  }

  event.respondWith(
    fetch(event.request)
      .then((response) => {
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(event.request, copy));
        return response;
      })
      .catch(() => caches.match(event.request).then((r) => r || caches.match("/")))
  );
});
)JS";

static const char APP_ICON_SVG[] PROGMEM = R"SVG(
<svg xmlns="http://www.w3.org/2000/svg" width="512" height="512" viewBox="0 0 512 512">
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#0d1a2b"/>
      <stop offset="100%" stop-color="#173645"/>
    </linearGradient>
    <linearGradient id="bolt" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#9cf9e3"/>
      <stop offset="100%" stop-color="#36d0a7"/>
    </linearGradient>
  </defs>
  <rect x="16" y="16" width="480" height="480" rx="110" fill="url(#bg)"/>
  <rect x="132" y="84" width="248" height="344" rx="86" fill="#112b3a" stroke="#4ecaa8" stroke-width="18"/>
  <rect x="176" y="118" width="160" height="28" rx="14" fill="#4ecaa8"/>
  <polygon points="292,176 226,270 274,270 220,352 320,238 270,238" fill="url(#bolt)"/>
</svg>
)SVG";

static const char MAIN_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>RotosisEVSE Panel</title>
<style>
body{font-family:Arial;margin:0;background:#0b1220;color:#e8eefc}
.wrap{display:grid;grid-template-columns:1.2fr .8fr;gap:12px;padding:12px}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
.card{background:#111a2b;border:1px solid #20304a;border-radius:12px;padding:12px}
h2{margin:0 0 10px 0;font-size:14px;color:#b7c5e6}
.kv{display:grid;grid-template-columns:140px 1fr;gap:8px;margin:6px 0}
.k{color:#b7c5e6;font-size:12px}
.v{font-weight:700}
.mono{font-family:monospace;color:#00ffaa}
input{width:100%;padding:8px;border-radius:10px;border:1px solid #20304a;background:#0c1424;color:#e8eefc}
.btns{display:flex;flex-wrap:wrap;gap:8px}
button{padding:8px 10px;border-radius:10px;border:1px solid #20304a;background:#0c1424;color:#e8eefc;cursor:pointer}
.primary{background:rgba(0,200,150,.18);border-color:rgba(0,200,150,.45)}
.danger{background:rgba(255,77,77,.14);border-color:rgba(255,77,77,.40)}
.small{font-size:11px;color:#b7c5e6}
.sep{height:1px;background:#20304a;margin:10px 0}

.hero{background:linear-gradient(180deg, rgba(0,200,150,.10), rgba(0,0,0,0));border:1px solid #20304a;border-radius:12px;padding:10px;margin-bottom:10px}
.heroTop{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:6px}
.badge{font-family:monospace;font-size:12px;padding:4px 8px;border-radius:999px;border:1px solid rgba(0,200,150,.45);background:rgba(0,200,150,.12);color:#00ffaa}
.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.grid2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
@media(max-width:680px){.grid3,.grid2{grid-template-columns:1fr}}
.hintBox{padding:10px 12px;border-radius:12px;background:rgba(0,200,150,.08);border:1px solid rgba(0,200,150,.18);color:#bde9dd;font-size:12px;line-height:1.55}

</style></head><body>

<div class="wrap">
  <div class="card">
    <h2>CANLI VERÄ°LER</h2>
    <div class="kv"><div class="k">State (Stable/Raw)</div><div class="v mono"><span id="sStb">-</span> / <span id="sRaw">-</span></div></div>
    <div class="kv"><div class="k">CP High / Low</div><div class="v mono"><span id="cH">-</span> / <span id="cL">-</span></div></div>
    <div class="kv"><div class="k">ADC High / Low</div><div class="v mono"><span id="aH">-</span> / <span id="aL">-</span></div></div>

    <div class="sep"></div>
    <h2>ZAMANLAMA</h2>
    <div class="kv"><div class="k">Loop dongusu (ms)</div><div class="v"><input id="lInt" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Role acma (ms)</div><div class="v"><input id="onD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Role birakma (ms)</div><div class="v"><input id="offD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Stable count</div><div class="v"><input id="stb" onfocus="p()" onblur="r()"></div></div>

    <div class="sep"></div>
    <h2>CP KALIBRASYON</h2>
    <div class="kv"><div class="k">Divider</div><div class="v"><input id="div" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">TH_B / TH_C</div><div class="v grid2"><input id="thb" onfocus="p()" onblur="r()"><input id="thc" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">TH_D / TH_E</div><div class="v grid2"><input id="thd" onfocus="p()" onblur="r()"><input id="the" onfocus="p()" onblur="r()"></div></div>

    <div class="sep"></div>
    <h2>AKIM KALIBRASYONU</h2>
    <div class="hintBox">
      0-10A araliginda varsayilan kalibrasyon 12.0. 10-30A araliginda varsayilan ek offset 1.0A.
      Buradan hem faz bazli kalibrasyon katsayilarini hem de orta aralik ofsetini degistirebilirsin.
    </div>
    <div class="kv"><div class="k">Ical A / B / C</div><div class="v grid3"><input id="icalA" onfocus="p()" onblur="r()"><input id="icalB" onfocus="p()" onblur="r()"><input id="icalC" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Offset A / B / C</div><div class="v grid3"><input id="ioffA" onfocus="p()" onblur="r()"><input id="ioffB" onfocus="p()" onblur="r()"><input id="ioffC" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">0-10A profil</div><div class="v grid2"><input id="rngLowMax" onfocus="p()" onblur="r()" placeholder="max A"><input id="rngLowOff" onfocus="p()" onblur="r()" placeholder="offset A"></div></div>
    <div class="kv"><div class="k">10-30A profil</div><div class="v grid2"><input id="rngMidMax" onfocus="p()" onblur="r()" placeholder="max A"><input id="rngMidOff" onfocus="p()" onblur="r()" placeholder="offset A"></div></div>
    <div class="small">Varsayilan tavsiye: `rngLowMax=10`, `rngLowOff=0`, `rngMidMax=30`, `rngMidOff=1`.</div>

    <div class="sep"></div>
    <h2>PENS AMPERMETRE YARDIMI</h2>
    <div class="kv"><div class="k">Canli A / B / C</div><div class="v mono"><span id="liveIa">-</span> / <span id="liveIb">-</span> / <span id="liveIc">-</span> A</div></div>
    <div class="kv"><div class="k">Canli ortalama</div><div class="v mono"><span id="liveIAvg">-</span> A</div></div>
    <div class="kv"><div class="k">Pens 0-10A A / B / C</div><div class="v grid3"><input id="clampLowA" onfocus="p()" onblur="r()" placeholder="A"><input id="clampLowB" onfocus="p()" onblur="r()" placeholder="B"><input id="clampLowC" onfocus="p()" onblur="r()" placeholder="C"></div></div>
    <div class="btns"><button onclick="fillClampLow()">Pens A/B/C ile Ical doldur</button></div>
    <div class="kv"><div class="k">Pens 10-30A A / B / C</div><div class="v grid3"><input id="clampMidA" onfocus="p()" onblur="r()" placeholder="A"><input id="clampMidB" onfocus="p()" onblur="r()" placeholder="B"><input id="clampMidC" onfocus="p()" onblur="r()" placeholder="C"></div></div>
    <div class="btns"><button onclick="fillClampMid()">Pens A/B/C ile offset doldur</button></div>
    <div class="small">Ilk buton her faz icin girilen pens degerine gore `Ical A/B/C` alanlarini ayarlar. Ikinci buton her fazin pens-ekran farkina gore `Offset A/B/C` alanlarini doldurur.</div>

    <div class="btns" style="margin-top:12px">
      <button class="primary" onclick="applyCal()">UYGULA</button>
      <button onclick="pull(true)">YENILE</button>
      <button onclick="window.location='/'">KULLANICI EKRANI</button>
    </div>
  </div>

  <div class="card">
    <div class="hero" id="displayPanel">
  <div class="heroTop">
    <div style="display:flex;gap:8px;align-items:center">
      <h2 style="margin:0">RotosisEVSE</h2>
      <span class="badge" id="badgeState">STATE:-</span>
    </div>
    <span class="badge" id="badgeRelay">R: -</span>
  </div>
  <div class="kv"><div class="k">I1 / I2 / I3</div><div class="v mono"><span id="i1">-</span> / <span id="i2">-</span> / <span id="i3">-</span> A</div></div>
  <div class="kv"><div class="k">Power</div><div class="v mono"><span id="pwr">-</span> kW</div></div>
  <div class="kv"><div class="k">Energy</div><div class="v mono"><span id="ekwh">-</span> kWh</div></div>
  <div class="kv"><div class="k">Time</div><div class="v mono"><span id="tsec">-</span></div></div>
  <div class="kv"><div class="k">Phase</div><div class="v mono"><span id="phase">-</span></div></div>
  <div class="kv"><div class="k">Wi-Fi</div><div class="v mono"><span id="wifiSsid">-</span> (<span id="wifiLoc">-</span>)</div></div>
  <div class="kv"><div class="k">IP</div><div class="v mono"><span id="ip">-</span></div></div>
  <div class="kv"><div class="k">Host</div><div class="v mono"><span id="host">-</span></div></div>
  <div class="kv"><div class="k">Relay</div><div class="v mono"><span id="rLbl">-</span></div></div>
</div>
    </div>

    <div class="sep"></div>

    <h2>RÃ¶le</h2>
    <div class="btns">
      <button class="primary" onclick="send('/relay?on=1')">MANUEL AÃ‡</button>
      <button class="danger" onclick="send('/relay?on=0')">MANUEL BIRAK</button>
      <button onclick="send('/relay_auto?en=1')">AUTO AÃ‡</button>
    </div>
    <div class="small">Not: MANUEL komut AUTOâ€™yu kapatÄ±r.</div>
    <div class="sep"></div>

    <h2>MOSFET Test</h2>
    <div class="btns">
      <button class="danger" onclick="send('/pulse_reset')">RESET (GPIO 7)</button>
      <button class="primary" onclick="send('/pulse_set')">SET (GPIO 15)</button>
    </div>
    <div class="small">Not: 100ms HIGH darbe.</div>
    <div class="sep"></div>

    <h2>Sifirlama Gecmisi</h2>
    <div class="kv"><div class="k">Toplam</div><div class="v mono"><span id="rstTotal">0</span></div></div>
    <div class="kv"><div class="k">Anlik / Gecmis</div><div class="v mono"><span id="rstNow">0</span> / <span id="rstHist">0</span></div></div>
    <div class="kv"><div class="k">Son Islem</div><div class="v mono"><span id="rstLastMode">YOK</span> @ <span id="rstLastSec">0</span>s</div></div>

    <div class="sep"></div>
    <h2>OTA Teshis</h2>
    <div class="kv"><div class="k">FW / Remote</div><div class="v mono"><span id="otaCurVer">-</span> / <span id="otaRemoteVer">-</span></div></div>
    <div class="kv"><div class="k">Calisan bolum</div><div class="v mono"><span id="otaPart">-</span></div></div>
    <div class="kv"><div class="k">Image state</div><div class="v mono"><span id="otaImgState">-</span></div></div>
    <div class="kv"><div class="k">Durum</div><div class="v mono"><span id="otaStatus">-</span></div></div>
    <div class="kv"><div class="k">Son kontrol</div><div class="v mono"><span id="otaAge">-</span></div></div>
    <div class="kv"><div class="k">Hata</div><div class="v mono"><span id="otaError">Hata yok</span></div></div>
    <div class="btns" style="margin-top:10px">
      <button class="primary" onclick="window.location='/update'">BIN YUKLE</button>
      <button class="primary" onclick="runOtaCheckAdmin()">OTA KONTROL ET</button>
      <button onclick="runBootPrev()">ONCEKI OTA'YA DON</button>
      <button class="danger" onclick="runBootFactory()">FACTORY'E DON</button>
    </div>
    <div class="small">Not: GitHub kontrolu saatte bir otomatik calisir. `Factory'e don` USB ile yukledigin sabit kurtarma surumunu acar.</div>



  </div>
</div>


<script>
let paused = false; let t;
const inps = ["lInt","onD","offD","stb","div","thb","thc","thd","the"];
const keys = ["lInt","onD","offD","stable","div","thb","thc","thd","the"];

function p(){ paused=true; clearTimeout(t); }
function r(){ t=setTimeout(()=>{paused=false;},3000); }
function send(path){ fetch(path).then(_=>pull(false)); }
function num(v, fallback = 0){
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}
function setInput(id, value, force){
  const el = document.getElementById(id);
  if(el && (document.activeElement !== el || force)) el.value = value;
}

function fmtTime(sec){
  const m = Math.floor(sec/60);
  const s = sec%60;
  const mm = String(m).padStart(2, "0");
  const ss = String(s).padStart(2, "0");
  return mm + ":" + ss;
}

function fmtOtaAge(ms){
  const sec = Math.max(0, Math.round((Number(ms) || 0) / 1000));
  if(sec === 0) return "hemen simdi";
  if(sec < 60) return sec + " sn once";
  const min = Math.floor(sec / 60);
  const rem = sec % 60;
  return min + " dk " + rem + " sn once";
}

function runOtaCheckAdmin(){
  fetch('/ota_check', {cache:'no-store'}).then(() => {
    document.getElementById('otaStatus').textContent = 'check_requested';
  });
}

function runBootFactory(){
  if(!confirm('Factory surume donup cihaz yeniden baslatilsin mi?')) return;
  fetch('/boot_factory', {cache:'no-store'})
    .then(r => r.text().then(t => ({ok:r.ok, text:t})))
    .then(x => {
      if(!x.ok) throw new Error(x.text || 'Factory gecisi basarisiz');
      alert(x.text || 'Factory secildi, cihaz yeniden baslatiliyor.');
    })
    .catch(e => alert(e.message || 'Factory gecisi basarisiz'));
}

function runBootPrev(){
  if(!confirm('Aktif olmayan OTA slotu secilip cihaz yeniden baslatilsin mi?')) return;
  fetch('/boot_prev', {cache:'no-store'})
    .then(r => r.text().then(t => ({ok:r.ok, text:t})))
    .then(x => {
      if(!x.ok) throw new Error(x.text || 'Onceki OTA gecisi basarisiz');
      alert(x.text || 'Diger OTA slotu secildi, cihaz yeniden baslatiliyor.');
    })
    .catch(e => alert(e.message || 'Onceki OTA gecisi basarisiz'));
}

function fillClampLow(){
  let changed = 0;
  ['A','B','C'].forEach(ph => {
    const ref = num(document.getElementById('clampLow' + ph).value, 0);
    const live = num(document.getElementById('liveI' + ph).textContent, 0);
    const currentCal = num(document.getElementById('ical' + ph).value, 12);
    if(ref > 0 && live > 0.2){
      document.getElementById('ical' + ph).value = (currentCal * (ref / live)).toFixed(2);
      changed++;
    }
  });
  if(!changed){
    alert('Pens A/B/C referanslarindan en az birini gir ve canli akim gelsin.');
    return;
  }
  alert('Ical A/B/C alanlari pens faz degerlerine gore dolduruldu.');
}

function fillClampMid(){
  let changed = 0;
  ['A','B','C'].forEach(ph => {
    const ref = num(document.getElementById('clampMid' + ph).value, 0);
    const live = num(document.getElementById('liveI' + ph).textContent, 0);
    if(ref > 0 && live > 0){
      document.getElementById('ioff' + ph).value = (ref - live).toFixed(2);
      changed++;
    }
  });
  if(!changed){
    alert('Pens A/B/C referanslarindan en az birini gir ve canli akim gelsin.');
    return;
  }
  alert('Offset A/B/C alanlari pens faz degerlerine gore dolduruldu.');
}

function pull(force=false){
  if(paused && !force) return;
  fetch('/status', {cache:'no-store'}).then(r=>r.json()).then(d=>{
    document.getElementById('sStb').textContent = d.state;
    document.getElementById('sRaw').textContent = d.stateRaw;
    document.getElementById('cH').textContent = d.cpHigh.toFixed(2)+"V";
    document.getElementById('cL').textContent = d.cpLow.toFixed(2)+"V";
    document.getElementById('aH').textContent = d.adcHigh.toFixed(3)+"V";
    document.getElementById('aL').textContent = d.adcLow.toFixed(3)+"V";
    if(d.ia !== undefined) document.getElementById('i1').textContent = d.ia.toFixed(2);
    if(d.ib !== undefined) document.getElementById('i2').textContent = d.ib.toFixed(2);
    if(d.ic !== undefined) document.getElementById('i3').textContent = d.ic.toFixed(2);
    if(d.pW !== undefined) document.getElementById('pwr').textContent = (d.pW/1000.0).toFixed(2);
    if(d.eKWh !== undefined) document.getElementById('ekwh').textContent = d.eKWh.toFixed(3);
    if(d.tSec !== undefined) document.getElementById('tsec').textContent = fmtTime(d.tSec);
    if(d.phase !== undefined) document.getElementById('phase').textContent = d.phase + "F";
    if(d.wifiSsid !== undefined) document.getElementById('wifiSsid').textContent = d.wifiSsid;
    if(d.wifiLoc !== undefined) document.getElementById('wifiLoc').textContent = d.wifiLoc;
    if(d.ip !== undefined) document.getElementById('ip').textContent = d.ip;
    if(d.host !== undefined) document.getElementById('host').textContent = d.host;
    if(d.rLbl !== undefined) document.getElementById('rLbl').textContent = d.rLbl;
    if(d.rstTotal !== undefined) document.getElementById('rstTotal').textContent = d.rstTotal;
    if(d.rstNow !== undefined) document.getElementById('rstNow').textContent = d.rstNow;
    if(d.rstHist !== undefined) document.getElementById('rstHist').textContent = d.rstHist;
    if(d.rstLastMode !== undefined) document.getElementById('rstLastMode').textContent = d.rstLastMode;
    if(d.rstLastSec !== undefined) document.getElementById('rstLastSec').textContent = d.rstLastSec;
    document.getElementById('otaCurVer').textContent = d.otaCur || '-';
    document.getElementById('otaRemoteVer').textContent = d.otaRemote || '-';
    document.getElementById('otaPart').textContent = d.otaPart || '-';
    document.getElementById('otaImgState').textContent = d.otaImgState || '-';
    document.getElementById('otaStatus').textContent = d.otaStatus || '-';
    document.getElementById('otaAge').textContent = fmtOtaAge(d.otaAgeMs);
    document.getElementById('otaError').textContent = (d.otaErr && d.otaErr.length) ? d.otaErr : 'Hata yok';
    const liveA = num(d.ia);
    const liveB = num(d.ib);
    const liveC = num(d.ic);
    document.getElementById('liveIa').textContent = liveA.toFixed(2);
    document.getElementById('liveIb').textContent = liveB.toFixed(2);
    document.getElementById('liveIc').textContent = liveC.toFixed(2);
    const liveVals = [liveA, liveB, liveC].filter(v => v > 0.1);
    const liveAvg = liveVals.length ? (liveVals.reduce((sum, v) => sum + v, 0) / liveVals.length) : 0;
    document.getElementById('liveIAvg').textContent = liveAvg.toFixed(2);

    // Ust rozetler
    const st = (d.state || "-");
    document.getElementById('badgeState').textContent = "STATE:" + st;

    const relayBadge = document.getElementById('badgeRelay');
    const rtxt = d.rLbl ? d.rLbl : "-";
    relayBadge.textContent = "R: " + rtxt;

    // Sarjdayken kablo parlasin
    if(d.state === "C") {
      relayBadge.style.backgroundColor = "rgba(0,255,170,0.3)";
    } else {
      relayBadge.style.backgroundColor = "";
    }

    inps.forEach((id, i) => {
      let el = document.getElementById(id);
      if(el && (document.activeElement !== el || force)) el.value = d[keys[i]];
    });
    setInput('icalA', d.icalA, force);
    setInput('icalB', d.icalB, force);
    setInput('icalC', d.icalC, force);
    setInput('ioffA', d.ioffA, force);
    setInput('ioffB', d.ioffB, force);
    setInput('ioffC', d.ioffC, force);
    setInput('rngLowMax', d.rngLowMax, force);
    setInput('rngMidMax', d.rngMidMax, force);
    setInput('rngLowOff', d.rngLowOff, force);
    setInput('rngMidOff', d.rngMidOff, force);
  });
}

function applyCal(){
  const q = new URLSearchParams();
  inps.forEach((id, i) => q.append(keys[i]==="stable"?"s":keys[i], document.getElementById(id).value));
  q.append('icalA', document.getElementById('icalA').value);
  q.append('icalB', document.getElementById('icalB').value);
  q.append('icalC', document.getElementById('icalC').value);
  q.append('ioffA', document.getElementById('ioffA').value);
  q.append('ioffB', document.getElementById('ioffB').value);
  q.append('ioffC', document.getElementById('ioffC').value);
  q.append('rngLowMax', document.getElementById('rngLowMax').value);
  q.append('rngMidMax', document.getElementById('rngMidMax').value);
  q.append('rngLowOff', document.getElementById('rngLowOff').value);
  q.append('rngMidOff', document.getElementById('rngMidOff').value);
  if(document.activeElement) document.activeElement.blur();
  fetch('/calib_apply?'+q.toString()).then(_=>{alert("Tamam"); paused=false; pull(true);});
}

setInterval(()=>pull(false), 2000);
pull(true);
</script>

</body></html>
)HTML";

static void setupWiFi() {
  // Tanimli aglar arasinda tarayarak STA modunda baglan.
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.setHostname(kHostName);
  WiFi.mode(WIFI_STA);
  WiFi.softAPdisconnect(true);
  Serial.println("[WiFi] AP kapali, sadece STA modu aktif.");
  Serial.print("[WiFi] Hostname: ");
  Serial.println(kHostName);

  if (!wifiEventsReady) {
    wifiEventsReady = true;
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
      Serial.print("[WiFi] Event: ");
      Serial.println((int)event);
      if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
        Serial.print("[WiFi] Connected SSID: ");
        Serial.println(WiFi.SSID());
      } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        Serial.print("[WiFi] Disconnected, reason: ");
        Serial.println((int)info.wifi_sta_disconnected.reason);
      } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
        Serial.print("[WiFi] Got IP: ");
        Serial.println(WiFi.localIP());
      }
    });
  }

  Serial.println("\nWi-Fi aglari eklendi:");
  size_t addedWifiCount = 0;
  for (size_t i = 0; i < (sizeof(kKnownWifis) / sizeof(kKnownWifis[0])); i++) {
    if (!hasText(kKnownWifis[i].ssid)) continue;
    wifiMulti.addAP(kKnownWifis[i].ssid, kKnownWifis[i].password);
    addedWifiCount++;
    Serial.printf(" - %s (%s)\n", kKnownWifis[i].location, kKnownWifis[i].ssid);
  }
  if (addedWifiCount == 0) {
    Serial.println(" - STA listesi bos.");
  }

  if (addedWifiCount > 0 && wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi Baglandi!");
    Serial.print("Konum: ");
    Serial.println(wifiLocationForSsid(WiFi.SSID()));
    Serial.println("IP adresi: ");
    Serial.println(WiFi.localIP());
  }

  Serial.println("[WiFi] Kendi AP yayini kapali.");
  refreshMdns();
}

// 3) HTTP handler'lari.
// Her endpoint kendi verisini veya komutunu burada uretir.
static void handleRoot() { server.send_P(200, "text/html", USER_HTML); }
static void handleAdmin() {
  if (!requireAdminAuth()) return;
  server.send_P(200, "text/html", MAIN_HTML);
}
static void handlePing() { server.send(200, "text/plain", "OK"); }
static void handleManifest() { server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
static void handleServiceWorker() { server.send_P(200, "application/javascript", SERVICE_WORKER_JS); }
static void handleAppIcon() { server.send_P(200, "image/svg+xml", APP_ICON_SVG); }
static void handleOtaCheck() {
  OTA_Manager::triggerCheckNow();
  server.send(200, "application/json", "{\"ok\":1}");
}

static void scheduleDeferredRestart() {
  s_manualOtaRebootPending = true;
  s_manualOtaRebootAtMs = millis() + 300;
}

static void handleBootFactory() {
  if (!requireAdminAuth()) return;
  if (!OTA_Manager::selectFactoryBootPartition()) {
    server.send(500, "text/plain", OTA_Manager::lastErrorText());
    return;
  }
  scheduleDeferredRestart();
  server.send(200, "text/plain", "Factory secildi, cihaz yeniden baslatiliyor");
}

static void handleBootPrev() {
  if (!requireAdminAuth()) return;
  if (!OTA_Manager::selectAlternateOtaBootPartition()) {
    server.send(409, "text/plain", OTA_Manager::lastErrorText());
    return;
  }
  scheduleDeferredRestart();
  server.send(200, "text/plain", "Diger OTA slotu secildi, cihaz yeniden baslatiliyor");
}

static void failManualOta(const String& reason) {
  if (s_manualOta.updateBegun) {
    Update.abort();
    s_manualOta.updateBegun = false;
  }
  s_manualOta.success = false;
  s_manualOta.lastError = reason;
  Serial.printf("[OTA] Manual upload reddedildi: %s\n", reason.c_str());
}

static void handleManualUpdatePage() {
  if (!requireAdminAuth()) return;

  String html;
  html.reserve(1800);
  html += F("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Guvenli OTA Yukleme</title>"
            "<style>"
            "body{font-family:Arial,sans-serif;background:#f4f1ea;color:#1b1c1d;margin:0;padding:24px}"
            ".card{max-width:640px;margin:0 auto;background:#fff;border:1px solid #d9d2c5;border-radius:16px;padding:24px}"
            "h1{margin:0 0 12px;font-size:28px}.muted{color:#6b665c;line-height:1.5}"
            ".meta{margin:16px 0;padding:14px;background:#f7f3eb;border-radius:12px}"
            "input[type=file]{display:block;margin:18px 0;width:100%}"
            "button{border:0;border-radius:999px;padding:14px 20px;background:#184e3b;color:#fff;font-weight:700;cursor:pointer}"
            "a{color:#184e3b;text-decoration:none;font-weight:700}"
            "</style></head><body><div class='card'>");
  html += F("<h1>Guvenli OTA Yukleme</h1>");
  html += F("<div class='muted'>Bu ekran uygulama firmware .bin dosyalarini kabul eder. "
            "USB disinda yazma sadece OTA slotlarina yapilir; factory bolumu korunur.</div>");
  html += F("<div class='meta'><b>Calisan surum:</b> ");
  html += OTA_Manager::currentVersion();
  html += F("<br><b>Yazma hedefi:</b> ota_0 / ota_1"
            "<br><b>Not:</b> USB haricinde factory bolumu yazilmaz.</div>");
  html += F("<form method='POST' action='/update' enctype='multipart/form-data'>"
            "<input type='file' name='firmware' accept='.bin,application/octet-stream' required>"
            "<button type='submit'>BIN YUKLE</button>"
            "</form><p class='muted'><a href='/admin'>Admin panele don</a></p>"
            "</div></body></html>");
  server.send(200, "text/html", html);
}

static void handleManualUpdateUpload() {
  if (!server.authenticate(kAdminUser, kAdminPassword)) {
    return;
  }

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    resetManualOtaState();
    s_manualOta.active = true;
    s_manualOta.uploadedName = upload.filename;

    const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
    if (!next ||
        (next->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_0 &&
         next->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_1)) {
      failManualOta("Guvenli hedef bulunamadi; sadece ota_0/ota_1 yazilabilir");
      return;
    }

    Serial.printf("[OTA] Manual upload basladi: %s\n", upload.filename.c_str());
    return;
  }

  if (!s_manualOta.active) return;

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (s_manualOta.lastError.length()) return;

    if (!s_manualOta.updateBegun) {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
        failManualOta(String("Update.begin hatasi: ") + Update.errorString());
        return;
      }
      s_manualOta.updateBegun = true;
      Serial.printf("[OTA] Manual upload ota slotuna kabul edildi: %s\n", s_manualOta.uploadedName.c_str());
    }

    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      failManualOta(String("Chunk yazma hatasi: ") + Update.errorString());
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (s_manualOta.lastError.length()) return;
    if (!s_manualOta.updateBegun) {
      failManualOta("OTA yazma baslatilamadi");
      return;
    }
    if (!Update.end(true)) {
      failManualOta(String("Update.end hatasi: ") + Update.errorString());
      return;
    }
    s_manualOta.success = true;
    Serial.printf("[OTA] Manual upload tamamlandi: %s (%u bytes)\n",
                  s_manualOta.uploadedName.c_str(),
                  (unsigned)upload.totalSize);
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    failManualOta("Yukleme iptal edildi");
    s_manualOta.active = false;
  }
}

static void handleManualUpdateResult() {
  if (!server.authenticate(kAdminUser, kAdminPassword)) {
    server.requestAuthentication(BASIC_AUTH, "RotosisEVSE Admin", "Sifre gerekli");
    return;
  }

  int code = s_manualOta.success ? 200 : 400;
  String body;
  const char* contentType = "text/plain";
  if (s_manualOta.success) {
    contentType = "text/html";
    body = F("<!doctype html><html><head><meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>OTA Tamam</title>"
             "<style>body{font-family:Arial,sans-serif;background:#f4f1ea;color:#1b1c1d;padding:24px}"
             ".card{max-width:560px;margin:0 auto;background:#fff;border:1px solid #d9d2c5;border-radius:16px;padding:24px}"
             "</style></head><body><div class='card'><h1>Yukleme Tamam</h1><p>Cihaz yeniden baslatiliyor.</p><p id='s'>Bekleniyor...</p>"
             "<script>"
             "setTimeout(function(){"
             "var tries=0;"
             "var t=setInterval(function(){"
             "tries++;"
             "fetch('/ping',{cache:'no-store'}).then(function(){clearInterval(t);location='/admin';})"
             ".catch(function(){document.getElementById('s').textContent='Cihaz tekrar aciliyor... ('+tries+')';});"
             "},1500);"
             "},1800);"
             "</script></div></body></html>");
    s_manualOtaRebootPending = true;
    s_manualOtaRebootAtMs = millis() + 1500;
  } else {
    body = s_manualOta.lastError.length() ? s_manualOta.lastError : "OTA yukleme basarisiz";
  }

  s_manualOta.active = false;
  server.send(code, contentType, body);
}

// Status endpoint'i kullanici panelinin ana veri kaynagidir.
static void handleStatus() {
  auto m = pilot_get();
  uint32_t nowMs = millis();

  float ia = safeFinite(current_sensor_get_irms_a());
  float ib = safeFinite(current_sensor_get_irms_b());
  float ic = safeFinite(current_sensor_get_irms_c());
  bool relaySet = relay_get();
  bool chargingState = (m.stateStable == "C" || m.stateStable == "D");
  bool accountingEnabled = relaySet && pwmEnabled && chargingState;
  if (!accountingEnabled) {
    ia = 0.0f;
    ib = 0.0f;
    ic = 0.0f;
  }
  float pW = accountingEnabled ? safeFinite(g_powerW) : 0.0f;
  float eKWh = safeFinite(g_energyKWh);
  float cpHigh = safeFinite(m.cpHigh);
  float cpLow = safeFinite(m.cpLow);
  float adcHigh = safeFinite(m.adcHigh);
  float adcLow = safeFinite(m.adcLow);
  const char* relayLabel = relaySet ? "SET" : "RESET";
  float calA = 0.0f, calB = 0.0f, calC = 0.0f;
  float offA = 0.0f, offB = 0.0f, offC = 0.0f;
  float rngLowMax = 0.0f, rngMidMax = 0.0f, rngLowOff = 0.0f, rngMidOff = 0.0f;
  current_sensor_get_calibration(&calA, &calB, &calC, &offA, &offB, &offC);
  current_sensor_get_range_profile(&rngLowMax, &rngMidMax, &rngLowOff, &rngMidOff);
  float iMax = ia;
  if (ib > iMax) iMax = ib;
  if (ic > iMax) iMax = ic;

  String wifiSsid = "-";
  String wifiLoc = "-";
  String ipStr = "-";
  String hostStr = String(kHostName) + ".local";
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
  const char* otaCurrent = OTA_Manager::currentVersion();
  const char* otaRemote = OTA_Manager::lastRemoteVersion();
  const char* otaPart = OTA_Manager::runningPartitionLabel();
  const char* otaImgState = OTA_Manager::runningImageStateLabel();
  const char* otaStatus = OTA_Manager::lastStatusText();
  const char* otaError = OTA_Manager::lastErrorText();
  uint32_t otaAgeMs = OTA_Manager::lastCheckAgeMs();
  if (staOk) {
    wifiSsid = WiFi.SSID();
    wifiLoc = wifiLocationForSsid(wifiSsid);
    ipStr = WiFi.localIP().toString();
  }

  int alarmLv = 0;
  const char* alarmTxt = "Sistem normal";
  bool manualStopAlertOn = (g_manualStopAlertUntilMs != 0 && ((int32_t)(g_manualStopAlertUntilMs - nowMs) > 0));
  if (manualStopAlertOn) {
    alarmLv = 2;
    alarmTxt = "Sarj manuel durduruldu";
  } else if (m.stateStable == "E" || m.stateStable == "F") {
    alarmLv = 2;
    alarmTxt = "Pilot hata durumu";
  } else if (iMax > (g_currentLimitA + 1.0f)) {
    alarmLv = 1;
    alarmTxt = "Akim limiti ustu";
  } else if (!staOk) {
    alarmLv = 1;
    alarmTxt = "Wi-Fi baglantisi yok";
  }

  snprintf(
    s_jsonBuf, sizeof(s_jsonBuf),
    "{\"lInt\":%d,\"onD\":%lu,\"offD\":%lu,\"stable\":%d,"
    "\"cpHigh\":%.2f,\"cpLow\":%.2f,\"adcHigh\":%.3f,\"adcLow\":%.3f,"
    "\"stateRaw\":\"%s\",\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,"
    "\"pW\":%.1f,\"eKWh\":%.3f,\"tSec\":%lu,\"phase\":%d,\"rLbl\":\"%s\","
    "\"wifiSsid\":\"%s\",\"wifiLoc\":\"%s\",\"ip\":\"%s\",\"host\":\"%s\","
    "\"state\":\"%s\",\"div\":%.3f,\"thb\":%.2f,\"thc\":%.2f,\"thd\":%.2f,\"the\":%.2f,"
    "\"icalA\":%.2f,\"icalB\":%.2f,\"icalC\":%.2f,\"ioffA\":%.2f,\"ioffB\":%.2f,\"ioffC\":%.2f,"
    "\"rngLowMax\":%.2f,\"rngMidMax\":%.2f,\"rngLowOff\":%.2f,\"rngMidOff\":%.2f,"
    "\"otaCur\":\"%s\",\"otaRemote\":\"%s\",\"otaPart\":\"%s\",\"otaImgState\":\"%s\",\"otaStatus\":\"%s\",\"otaAgeMs\":%lu,\"otaErr\":\"%s\","
    "\"modeId\":%d,\"mode\":\"%s\",\"limitA\":%.1f,\"staOk\":%d,"
    "\"alarmLv\":%d,\"alarmTxt\":\"%s\","
    "\"sLive\":%d,\"sLiveStart\":%lu,\"sLiveSec\":%lu,\"sLiveKWh\":%.3f,"
    "\"rstTotal\":%lu,\"rstNow\":%lu,\"rstHist\":%lu,\"rstLastSec\":%lu,\"rstLastMode\":\"%s\"}",
    loopIntervalMs,
    (unsigned long)relayOnDelayMs,
    (unsigned long)relayOffDelayMs,
    stableCount,
    cpHigh, cpLow, adcHigh, adcLow,
    m.stateRaw.c_str(),
    ia, ib, ic,
    pW, eKWh,
    (unsigned long)g_chargeSeconds,
    g_phaseCount,
    relayLabel,
    wifiSsid.c_str(),
    wifiLoc.c_str(),
    ipStr.c_str(),
    hostStr.c_str(),
    m.stateStable.c_str(),
    CP_DIVIDER_RATIO,
    TH_B_MIN, TH_C_MIN, TH_D_MIN, TH_E_MIN,
    calA, calB, calC, offA, offB, offC,
    rngLowMax, rngMidMax, rngLowOff, rngMidOff,
    otaCurrent, otaRemote, otaPart, otaImgState, otaStatus, (unsigned long)otaAgeMs, otaError,
    g_chargeMode,
    chargeModeLabel(g_chargeMode),
    safeFinite(g_currentLimitA),
    staOk ? 1 : 0,
    alarmLv,
    alarmTxt,
    g_sessionLive ? 1 : 0,
    (unsigned long)g_sessionLiveStartSec,
    (unsigned long)g_sessionLiveSeconds,
    safeFinite(g_sessionLiveEnergyKWh),
    (unsigned long)s_resetTotalCount,
    (unsigned long)s_resetNowCount,
    (unsigned long)s_resetHistoryCount,
    (unsigned long)s_resetLastSec,
    resetModeLabel(s_resetLastModeId)
  );

  server.send(200, "application/json", s_jsonBuf);
}

// Gecmis seanslar icin ayri JSON endpoint.
static void handleHistory() {
  char json[4096];
  int n = 0;
  n += snprintf(
    json + n, sizeof(json) - n,
    "{\"count\":%d,\"active\":{\"on\":%d,\"start\":%lu,\"sec\":%lu,\"kWh\":%.3f},\"items\":[",
    g_histCount,
    g_sessionLive ? 1 : 0,
    (unsigned long)g_sessionLiveStartSec,
    (unsigned long)g_sessionLiveSeconds,
    safeFinite(g_sessionLiveEnergyKWh)
  );

  int start = (g_histCount < 20) ? 0 : g_histHead;
  for (int i = 0; i < g_histCount && n < (int)sizeof(json) - 2; i++) {
    int idx = (start + i) % 20;
    n += snprintf(
      json + n, sizeof(json) - n,
      "%s{\"s\":%lu,\"d\":%lu,\"e\":%.3f,\"p\":%.1f,\"ph\":%u}",
      (i == 0) ? "" : ",",
      (unsigned long)g_histStartSec[idx],
      (unsigned long)g_histDurationSec[idx],
      safeFinite(g_histEnergyKWh[idx]),
      safeFinite(g_histAvgPowerW[idx]),
      (unsigned)g_histPhaseCount[idx]
    );
  }
  snprintf(json + n, sizeof(json) - n, "]}");
  server.send(200, "application/json", json);
}

// Kullanici panelindeki AUTO / START / STOP komutu burada islenir.
static void handleChargeCmd() {
  if (!server.hasArg("m")) {
    server.send(400, "text/plain", "missing m");
    return;
  }
  int mode = clampIntArg(server.arg("m"), 0, 2);
  g_chargeMode = mode;

  // "Sarji Durdur" isteginde bir sonraki loop'u beklemeden cikislari hemen kapat.
  if (mode == 2) {
    g_manualStopAlertUntilMs = millis() + 10000UL;
    g_manualStopAutoResumeAtMs = millis() + 60000UL;
    pwmEnabled = false;
    pwmDutyPercent = 0;
    pilot_apply_pwm();
    relay_force_off_now();
    digitalWrite(ERROR_LED_PIN, HIGH);
  } else {
    g_manualStopAlertUntilMs = 0;
    g_manualStopAutoResumeAtMs = 0;
  }

  server.send(200, "text/plain", "OK");
}

// Enerji ve gecmis sifirlama endpoint'i.
static void handleDataReset() {
  bool clearNow = true;
  bool clearHistory = true;
  if (server.hasArg("now")) {
    clearNow = (server.arg("now") != "0");
  }
  if (server.hasArg("hist")) {
    clearHistory = (server.arg("hist") != "0");
  }
  if (clearNow) {
    resetChargeData(clearHistory);
  } else if (clearHistory) {
    resetHistoryData();
  }
  noteResetEvent(clearNow, clearHistory);

  char json[96];
  snprintf(
    json, sizeof(json),
    "{\"ok\":1,\"now\":%d,\"hist\":%d}",
    clearNow ? 1 : 0,
    clearHistory ? 1 : 0
  );
  server.send(200, "application/json", json);
}

// Web admin panelinden gelen CP / relay / timing ayarlari burada uygulanir.
static void handleCalibApply() {
  if (!requireAdminAuth()) return;
  float calA = 0.0f, calB = 0.0f, calC = 0.0f;
  float offA = 0.0f, offB = 0.0f, offC = 0.0f;
  float rngLowMax = 0.0f, rngMidMax = 0.0f, rngLowOff = 0.0f, rngMidOff = 0.0f;
  current_sensor_get_calibration(&calA, &calB, &calC, &offA, &offB, &offC);
  current_sensor_get_range_profile(&rngLowMax, &rngMidMax, &rngLowOff, &rngMidOff);
  if (server.hasArg("lInt")) {
    loopIntervalMs = clampIntArg(server.arg("lInt"), 20, 2000);
  }
  if (server.hasArg("onD")) {
    relayOnDelayMs = (uint32_t)clampIntArg(server.arg("onD"), 0, 60000);
  }
  if (server.hasArg("offD")) {
    relayOffDelayMs = (uint32_t)clampIntArg(server.arg("offD"), 0, 60000);
  }
  if (server.hasArg("s")) {
    stableCount = clampIntArg(server.arg("s"), 1, 50);
  }
  if (server.hasArg("div")) {
    CP_DIVIDER_RATIO = clampFloatArg(server.arg("div"), 0.1f, 20.0f, CP_DIVIDER_RATIO);
  }
  if (server.hasArg("thb")) {
    TH_B_MIN = clampFloatArg(server.arg("thb"), 0.0f, 15.0f, TH_B_MIN);
  }
  if (server.hasArg("thc")) {
    TH_C_MIN = clampFloatArg(server.arg("thc"), 0.0f, 15.0f, TH_C_MIN);
  }
  if (server.hasArg("thd")) {
    TH_D_MIN = clampFloatArg(server.arg("thd"), 0.0f, 15.0f, TH_D_MIN);
  }
  if (server.hasArg("the")) {
    TH_E_MIN = clampFloatArg(server.arg("the"), 0.0f, 15.0f, TH_E_MIN);
  }
  if (server.hasArg("icalA")) {
    calA = clampFloatArg(server.arg("icalA"), 1.0f, 80.0f, calA);
  }
  if (server.hasArg("icalB")) {
    calB = clampFloatArg(server.arg("icalB"), 1.0f, 80.0f, calB);
  }
  if (server.hasArg("icalC")) {
    calC = clampFloatArg(server.arg("icalC"), 1.0f, 80.0f, calC);
  }
  if (server.hasArg("ioffA")) {
    offA = clampFloatArg(server.arg("ioffA"), -10.0f, 10.0f, offA);
  }
  if (server.hasArg("ioffB")) {
    offB = clampFloatArg(server.arg("ioffB"), -10.0f, 10.0f, offB);
  }
  if (server.hasArg("ioffC")) {
    offC = clampFloatArg(server.arg("ioffC"), -10.0f, 10.0f, offC);
  }
  if (server.hasArg("rngLowMax")) {
    rngLowMax = clampFloatArg(server.arg("rngLowMax"), 1.0f, 80.0f, rngLowMax);
  }
  if (server.hasArg("rngMidMax")) {
    rngMidMax = clampFloatArg(server.arg("rngMidMax"), rngLowMax, 80.0f, rngMidMax);
  }
  if (server.hasArg("rngLowOff")) {
    rngLowOff = clampFloatArg(server.arg("rngLowOff"), -10.0f, 10.0f, rngLowOff);
  }
  if (server.hasArg("rngMidOff")) {
    rngMidOff = clampFloatArg(server.arg("rngMidOff"), -10.0f, 10.0f, rngMidOff);
  }
  current_sensor_set_calibration(calA, calB, calC, offA, offB, offC);
  current_sensor_set_range_profile(rngLowMax, rngMidMax, rngLowOff, rngMidOff);
  server.send(200, "text/plain", "OK");
}

static void handleRelay() {
  if (server.hasArg("on")) relay_set(server.arg("on") == "1");
  server.send(200, "text/plain", "OK");
}

static void handleRelayAuto() {
  if (server.hasArg("en")) relay_set_auto_enabled(server.arg("en") == "1");
  server.send(200, "text/plain", "OK");
}

static void handlePulseReset() {
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_RESET_PIN);
  server.send(200, "text/plain", "OK");
}

static void handlePulseSet() {
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_SET_PIN);
  server.send(200, "text/plain", "OK");
}


// 4) Route kayitlari ve servis baslatma.
void web_init() {
  // Boot sirasinda ag, OTA, route ve web server bu noktada ayaga kalkar.
  setupWiFi();
  setupArduinoOta();
  s_resetPrefsReady = s_resetPrefs.begin("evse", false);
  if (s_resetPrefsReady) {
    loadResetStats();
  } else {
    Serial.println("[RST] NVS init fail");
  }
  pinMode(MOSFET_RESET_PIN, OUTPUT);
  pinMode(MOSFET_SET_PIN, OUTPUT);
  digitalWrite(MOSFET_RESET_PIN, LOW);
  digitalWrite(MOSFET_SET_PIN, LOW);
  // HTTP route kayitlari.
  server.on("/", HTTP_GET, handleRoot);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/settings", HTTP_GET, handleAdmin);
  server.on("/update", HTTP_GET, handleManualUpdatePage);
  server.on("/update", HTTP_POST, handleManualUpdateResult, handleManualUpdateUpload);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/sw.js", HTTP_GET, handleServiceWorker);
  server.on("/app-icon.svg", HTTP_GET, handleAppIcon);
  server.on("/ota_check", HTTP_GET, handleOtaCheck);
  server.on("/boot_factory", HTTP_GET, handleBootFactory);
  server.on("/boot_prev", HTTP_GET, handleBootPrev);
  // Captive portal probe endpoints (Android/iOS/Windows)
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/data_reset", HTTP_GET, handleDataReset);
  server.on("/charge_cmd", HTTP_GET, handleChargeCmd);
  server.on("/calib_apply", HTTP_GET, handleCalibApply);
  server.on("/relay", HTTP_GET, handleRelay);
  server.on("/relay_auto", HTTP_GET, handleRelayAuto);
  server.on("/pulse_reset", HTTP_GET, handlePulseReset);
  server.on("/pulse_set", HTTP_GET, handlePulseSet);
  server.onNotFound(handleRoot);
  server.begin();
  s_serverStarted = true;
}

void web_loop() {
  // Arka plan servisleri her loop'ta buradan yurutulur.
  s_lastWebLoopMs = millis();
  server.handleClient();

  if (s_manualOtaRebootPending && (int32_t)(millis() - s_manualOtaRebootAtMs) >= 0) {
    s_manualOtaRebootPending = false;
    Serial.println("[BOOTCTL] Web komutu sonrasi yeniden baslatiliyor");
    delay(100);
    ESP.restart();
  }

  // WiFi yeniden baglanti denemesi: seyrek ve kisa timeout ile.
  static uint32_t lastWifiTryMs = 0;
  const uint32_t nowTry = millis();
  if (WiFi.status() != WL_CONNECTED && (nowTry - lastWifiTryMs >= 30000)) {
    lastWifiTryMs = nowTry;
    wifiMulti.run(120);
  }

  static bool printed = false;
  static uint32_t lastStatusPrintMs = 0;

  if (WiFi.status() == WL_CONNECTED) {
    if (!printed) {
      printed = true;
      Serial.print("STA SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("STA IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("HOST: http://");
      Serial.print(kHostName);
      Serial.println(".local");
    }
  } else {
    printed = false; 
    const uint32_t now = millis();
    if (now - lastStatusPrintMs >= 30000) {
      lastStatusPrintMs = now;
      Serial.print("[WiFi] Status: ");
      Serial.println((int)WiFi.status());
    }
  }

  refreshMdns();
}

bool web_ready_for_ota_validation() {
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
  if (!staOk || !s_serverStarted || s_lastWebLoopMs == 0) return false;
  return (millis() - s_lastWebLoopMs) < 1500;
}

