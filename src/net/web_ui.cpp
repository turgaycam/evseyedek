#include "web_ui.h"
#include <WiFi.h>
#include <WiFiMulti.h>

#include <WebServer.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
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
static const char* kOtaHostname = EVSE_OTA_HOSTNAME;
static const char* kOtaPassword = EVSE_OTA_PASSWORD;

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
static uint32_t s_lastHttpRequestMs = 0;
static uint32_t s_successfulHttpResponses = 0;
static uint32_t s_resetTotalCount = 0;
static uint32_t s_resetNowCount = 0;
static uint32_t s_resetHistoryCount = 0;
static uint32_t s_resetLastSec = 0;
static uint8_t s_resetLastModeId = 0;
static Preferences s_resetPrefs;
static bool s_resetPrefsReady = false;

static bool hasText(const char* value) {
  return value != nullptr && value[0] != '\0';
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
    MDNS.addService("arduino", "tcp", 3232);
    mdnsStarted = true;
    Serial.print("[mDNS] Ready: http://");
    Serial.print(kHostName);
    Serial.println(".local");
  }
}

static void setupArduinoOta() {
  ArduinoOTA.setHostname(kOtaHostname);
  if (hasText(kOtaPassword)) {
    ArduinoOTA.setPassword(kOtaPassword);
  }

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] End");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint32_t lastPct = 101;
    uint32_t pct = (total > 0) ? ((progress * 100U) / total) : 0U;
    if (pct != lastPct && (pct % 10U == 0U || pct == 100U)) {
      Serial.printf("[OTA] Progress: %u%%\n", (unsigned)pct);
      lastPct = pct;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]\n", (unsigned)error);
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready (port 3232)");
  Serial.print("[OTA] Hostname: ");
  Serial.println(kOtaHostname);
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

static void noteWebActivity() {
  // Kullanici aktifken periyodik OTA kontrolu web sunucusunu bloklamasin.
  s_lastHttpRequestMs = millis();
  OTA_Manager::deferPeriodicChecks(15000);
}

static void noteHttpResponseSent() {
  s_successfulHttpResponses++;
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
:root{
  --bg0:#071017;
  --bg1:#0c1d29;
  --panel:#102635e8;
  --panel2:#133043;
  --line:#2f6072;
  --text:#f1fbff;
  --muted:#9dc0ce;
  --accent:#4fe7b1;
  --accentSoft:#bfffe6;
  --warn:#ffc861;
  --err:#ff8b7c;
  --sky:#72d7ff;
  --glow:#59f3ce66;
}
html,body{margin:0;padding:0}
body{
  font-family:"Avenir Next","Trebuchet MS","Segoe UI",sans-serif;
  color:var(--text);
  min-height:100vh;
  background:
    radial-gradient(560px 280px at 50% -10%, #24445b 0%, transparent 70%),
    radial-gradient(440px 240px at 12% 15%, #16384c 0%, transparent 72%),
    linear-gradient(180deg, var(--bg0), var(--bg1) 62%, #08131b);
}
body.state-A{--accent:var(--sky);--accentSoft:#c7f3ff;--glow:#72d7ff55}
body.state-B{--accent:#8cbaff;--accentSoft:#dce8ff;--glow:#8cbaff55}
body.state-C,body.state-D{--accent:#4fe7b1;--accentSoft:#c5ffe5;--glow:#59f3ce66}
body.state-E,body.state-F{--accent:var(--err);--accentSoft:#ffd0c9;--glow:#ff8b7c55}
.app{
  max-width:430px;
  margin:0 auto;
  padding:16px 14px 18px;
}
.top{
  display:flex;
  justify-content:space-between;
  align-items:center;
  gap:10px;
  margin-bottom:12px;
}
.brand{
  font-size:28px;
  font-weight:700;
  letter-spacing:.2px;
}
.brandSub{
  margin-top:2px;
  color:var(--muted);
  font-size:12px;
}
.sync{
  border:1px solid var(--accent);
  background:#133547;
  color:var(--accentSoft);
  border-radius:999px;
  padding:7px 11px;
  font-size:12px;
  font-family:"Consolas","Lucida Console",monospace;
}
.sync.wait{
  border-color:#866f42;
  color:#ffdca0;
  background:#352715;
}
.card{
  border:1px solid var(--line);
  border-radius:28px;
  background:linear-gradient(180deg,var(--panel),#0e2230);
  box-shadow:0 20px 40px #00000030;
}
.hero{
  padding:16px 16px 18px;
  overflow:hidden;
  position:relative;
}
.hero::after{
  content:"";
  position:absolute;
  width:180px;
  height:180px;
  right:-55px;
  top:-55px;
  border-radius:999px;
  background:radial-gradient(circle,var(--accent) 0%, transparent 72%);
  opacity:.18;
}
.heroHead{
  display:grid;
  grid-template-columns:minmax(0,1fr) auto;
  gap:12px;
  align-items:start;
}
.chipRow{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:8px;
}
.chip{
  border:1px solid #3f6f83;
  border-radius:999px;
  padding:7px 8px;
  text-align:center;
  font-size:11px;
  font-family:"Consolas","Lucida Console",monospace;
  background:#112d3d;
  color:#ddf6ff;
}
.chip.main{
  border-color:var(--accent);
  color:var(--accentSoft);
}
.stateVisual{
  position:relative;
  width:62px;
  height:62px;
  border-radius:22px;
  border:1px solid #3f6f83;
  background:linear-gradient(180deg,#123243,#0f2432);
  display:grid;
  place-items:center;
  overflow:hidden;
  box-shadow:inset 0 1px 0 #ffffff12;
}
.stateVisual::before{
  content:"";
  position:absolute;
  inset:-18px;
  border-radius:999px;
  background:radial-gradient(circle,var(--accent) 0%, transparent 70%);
  opacity:0;
  transition:opacity .35s ease;
}
.stateGlyph{
  position:relative;
  width:38px;
  height:38px;
  transition:transform .35s ease,filter .35s ease;
}
.stateGlyphRing{
  fill:none;
  stroke:#60889b;
  stroke-width:3;
  opacity:.78;
  transition:stroke .35s ease,opacity .35s ease;
}
.stateGlyphBolt{
  fill:#d6edf6;
  opacity:.88;
  transition:fill .35s ease,opacity .35s ease;
}
body.state-B .stateVisual,
body.state-C .stateVisual,
body.state-D .stateVisual{
  border-color:var(--accent);
  box-shadow:inset 0 1px 0 #ffffff18,0 0 18px var(--glow);
}
body.state-B .stateVisual::before,
body.state-C .stateVisual::before,
body.state-D .stateVisual::before{
  opacity:.28;
}
body.state-B .stateGlyphRing,
body.state-C .stateGlyphRing,
body.state-D .stateGlyphRing{
  stroke:var(--accent);
  opacity:1;
}
body.state-B .stateGlyphBolt,
body.state-C .stateGlyphBolt,
body.state-D .stateGlyphBolt{
  fill:var(--accentSoft);
  opacity:1;
}
body.state-C .stateGlyph,
body.state-D .stateGlyph{
  animation:pulseIcon 1.8s ease-in-out infinite;
}
.heroGrid{
  margin-top:14px;
  display:grid;
  grid-template-columns:minmax(0,1fr) 188px;
  gap:14px;
  align-items:center;
}
.heroCopy{
  min-width:0;
}
.eyebrow{
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:1px;
}
.stateName{
  margin-top:6px;
  font-size:25px;
  font-weight:700;
  line-height:1.04;
}
.stateHint{
  margin-top:4px;
  color:var(--muted);
  font-size:14px;
}
.heroMiniRow{
  margin-top:14px;
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:10px;
}
.heroMini{
  padding:11px 12px;
  border:1px solid #31586b;
  border-radius:18px;
  background:#0f2431b8;
  min-width:0;
}
.heroMini span{
  display:block;
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.65px;
}
.heroMini strong{
  display:block;
  margin-top:6px;
  font-size:18px;
  line-height:1.05;
  font-family:"Consolas","Lucida Console",monospace;
}
.gaugeWrap{
  position:relative;
  width:188px;
  height:188px;
  justify-self:center;
}
.gaugeShell{
  position:absolute;
  inset:18px;
  border-radius:999px;
  background:radial-gradient(circle at 50% 34%, #1a4153 0%, #0f2533 56%, #09151d 100%);
  box-shadow:inset 0 1px 0 #ffffff18,inset 0 -16px 30px #0000002e;
}
.gaugeSvg{
  position:relative;
  width:100%;
  height:100%;
  transform:rotate(-90deg);
}
.gaugeTrack{
  fill:none;
  stroke:#214253;
  stroke-width:12;
}
.gaugeProgress{
  fill:none;
  stroke:url(#gaugeGradient);
  stroke-width:12;
  stroke-linecap:round;
  transition:stroke-dashoffset .42s ease;
  filter:drop-shadow(0 0 10px var(--glow));
}
.gaugeCenter{
  position:absolute;
  inset:0;
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  padding:0 18px;
  text-align:center;
}
.currentLine{
  display:flex;
  align-items:flex-end;
  gap:6px;
}
.currentValue{
  font-size:48px;
  line-height:.9;
  font-weight:700;
  letter-spacing:-1.8px;
  font-family:"Avenir Next Condensed","Trebuchet MS",sans-serif;
}
.currentUnit{
  margin-bottom:7px;
  color:var(--accentSoft);
  font-size:18px;
  font-weight:700;
}
.currentSub{
  margin-top:6px;
  color:var(--muted);
  font-size:12px;
}
.gaugeMeta{
  margin-top:10px;
}
.gaugeMeta span{
  display:block;
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.7px;
}
.gaugeMeta strong{
  display:block;
  margin-top:4px;
  font-size:20px;
  font-family:"Consolas","Lucida Console",monospace;
}
.actions{
  margin-top:12px;
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:10px;
}
.btn{
  border:1px solid #3c6d7f;
  border-radius:18px;
  padding:13px 10px;
  background:#133243;
  color:var(--text);
  text-decoration:none;
  text-align:center;
  font-size:15px;
  display:flex;
  align-items:center;
  justify-content:center;
  gap:8px;
  font-weight:600;
}
.btn.ok{background:#15473b;border-color:#4fc49e}
.btn.stop{background:#4e262d;border-color:#d37c75}
.btn.ghost{background:#143243;border-color:#5a8395}
.btnIcon{
  font-size:16px;
  line-height:1;
}
.btn.install{
  grid-column:1 / -1;
  display:none;
}
.stats{
  margin-top:12px;
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:10px;
  align-items:stretch;
}
.stat{
  padding:13px 12px;
  border:1px solid var(--line);
  border-radius:22px;
  background:linear-gradient(180deg,var(--panel2),#10293a);
  text-align:left;
}
.statLabel{
  display:flex;
  align-items:center;
  gap:7px;
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.7px;
}
.statIcon{
  width:24px;
  height:24px;
  display:grid;
  place-items:center;
  border-radius:999px;
  background:#ffffff12;
  color:var(--accentSoft);
  font-style:normal;
  font-size:13px;
}
.stat strong{
  display:block;
  margin-top:10px;
  font-size:25px;
  line-height:1.05;
  font-family:"Consolas","Lucida Console",monospace;
}
.stat strong em{
  font-style:normal;
  font-size:.5em;
  color:#cde9f4;
}
.stat small{
  display:block;
  margin-top:5px;
  color:#c9e3ee;
  font-size:11px;
}
.carCard{margin-top:12px;padding:16px 16px 18px}
.carWrap{position:relative;width:100%;max-width:320px;margin:0 auto 10px;display:flex;align-items:center;justify-content:center}
.carFill{position:absolute;inset:0;--fillScale:0;--fillColor:rgba(66,205,112,.9);--alpha:0;--car-img:none;-webkit-mask-image:var(--car-img);mask-image:var(--car-img);-webkit-mask-size:contain;mask-size:contain;-webkit-mask-repeat:no-repeat;mask-repeat:no-repeat;-webkit-mask-position:center;mask-position:center;overflow:hidden;opacity:var(--alpha);transition:opacity 1.2s ease,filter .35s ease}
.carFillInner{position:absolute;inset:0;background:var(--fillColor);transform-origin:bottom;height:0%;transition:background-color .9s ease}
.carFill.stateA{--alpha:0}
.carFill.stateB{--alpha:1;--fillColor:rgba(255,193,7,.85)}
.carFill.stateC{--alpha:1;--fillColor:rgba(76,175,80,.9)}
.carFill.stateC .carFillInner{animation:carChargeFill 2.2s ease-in-out infinite}
.carFill.stateD{--alpha:1;--fillColor:rgba(76,175,80,.85)}
.carFill.stateD .carFillInner{animation:carChargeFill 2.2s ease-in-out infinite}
.carFill.stateE{--alpha:1;--fillColor:rgba(244,67,54,.95)}
.carFill.stateE .carFillInner{animation:carFaultPulse 0.6s ease-in-out infinite}
.carFill.stateF{--alpha:1;--fillColor:rgba(211,47,47,.95)}
.carFill.stateF .carFillInner{animation:carFaultPulse 0.5s ease-in-out infinite}
.carSvg{width:100%;max-width:320px;height:auto;display:block;filter:drop-shadow(0 12px 28px rgba(0,0,0,.25))}
.batteryRow{display:flex;align-items:center;gap:8px;padding:0 4px 6px;font-weight:700;font-size:15px}
.batteryRow span:last-child{margin-left:auto;font-size:16px}
.badgeIcon{width:22px;height:22px;border-radius:6px;background:#0f2431;display:grid;place-items:center;font-size:14px;color:#b7d9e8;border:1px solid #2d5567}
.chargeBar{position:relative;margin:10px 4px 2px;height:8px;border-radius:999px;background:#102535;overflow:hidden;border:1px solid #203b4a}
.chargeBarInner{position:absolute;inset:0;background:linear-gradient(90deg,var(--accent),var(--accentSoft));transform-origin:left center;transform:scaleX(.3);transition:transform .4s ease}
.chargeBar.charging .chargeBarInner{animation:barMove 1.6s linear infinite}
@keyframes carChargeFill{0%{bottom:0%}50%{bottom:80%}100%{bottom:0%}}
@keyframes carFaultPulse{0%{filter:brightness(1.3) drop-shadow(0 0 8px rgba(244,67,54,0.8))}50%{filter:brightness(0.8) drop-shadow(0 0 2px rgba(244,67,54,0.4))}100%{filter:brightness(1.3) drop-shadow(0 0 8px rgba(244,67,54,0.8))}}
@keyframes barMove{0%{transform:scaleX(var(--pct,0.3)) translateX(-4%)}100%{transform:scaleX(var(--pct,0.3)) translateX(4%)}}
.chartCard{
  margin-top:12px;
  padding:16px;
}
.sectionHead{
  display:flex;
  justify-content:space-between;
  gap:12px;
  align-items:flex-end;
}
.sectionTitle{
  margin-top:4px;
  font-size:18px;
  font-weight:700;
}
.sectionValue{
  font-size:18px;
  color:var(--accentSoft);
  font-family:"Consolas","Lucida Console",monospace;
}
.chartFrame{
  margin-top:14px;
  height:172px;
  border:1px solid #31586a;
  border-radius:22px;
  background:linear-gradient(180deg,#0c1f2c,#0a1822);
  overflow:hidden;
}
.chartSvg{
  display:block;
  width:100%;
  height:100%;
}
.chartGrid{
  stroke:#ffffff14;
  stroke-width:1;
}
.chartArea{
  fill:url(#chartFill);
  opacity:.9;
}
.chartLine{
  fill:none;
  stroke:var(--accent);
  stroke-width:4;
  stroke-linecap:round;
  stroke-linejoin:round;
  filter:drop-shadow(0 0 8px var(--glow));
}
.chartDot{
  fill:var(--accentSoft);
  stroke:var(--accent);
  stroke-width:4;
}
.chartLegend{
  margin-top:12px;
  display:grid;
  grid-template-columns:repeat(3,minmax(0,1fr));
  gap:8px;
}
.chartLegend span{
  display:block;
  padding:10px 12px;
  border:1px solid #2c5567;
  border-radius:16px;
  background:#0f2431cc;
  color:var(--muted);
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.6px;
}
.chartLegend strong{
  display:block;
  margin-top:4px;
  color:var(--text);
  font-size:15px;
  text-transform:none;
  letter-spacing:0;
  font-family:"Consolas","Lucida Console",monospace;
}
.statusCard{
  margin-top:12px;
  padding:14px 14px 16px;
}
.statusCard.ok{
  border-color:#46a886;
  background:linear-gradient(180deg,#163c33,#112820);
}
.statusCard.warn{
  border-color:#b48b4b;
  background:linear-gradient(180deg,#43311d,#322414);
}
.statusCard.err{
  border-color:#c06862;
  background:linear-gradient(180deg,#49242b,#34181f);
}
.statusHead{
  display:flex;
  justify-content:space-between;
  gap:12px;
  align-items:center;
}
.statusBadge{
  display:inline-flex;
  align-items:center;
  gap:8px;
  padding:8px 10px;
  border:1px solid #ffffff22;
  border-radius:999px;
  background:#ffffff10;
  font-size:12px;
  color:#e8f7ff;
}
.statusOrb{
  width:10px;
  height:10px;
  border-radius:999px;
  background:#6fd6b5;
  box-shadow:0 0 10px #6fd6b577;
}
.statusOrb.warn{
  background:var(--warn);
  box-shadow:0 0 10px #ffc86177;
}
.statusOrb.err{
  background:var(--err);
  box-shadow:0 0 10px #ff8b7c77;
}
.statusText{
  margin-top:12px;
  font-size:24px;
  font-weight:700;
}
.statusMeta{
  margin-top:5px;
  font-size:13px;
  color:#ecf7fb;
}
.phaseRow{
  margin-top:14px;
  display:grid;
  grid-template-columns:repeat(3,minmax(0,1fr));
  gap:10px;
}
.phaseChip{
  border:1px solid #3c697b;
  border-radius:20px;
  padding:12px 10px;
  background:#102a38cc;
  text-align:center;
}
.phaseChip span{
  display:block;
  color:#b7d9e8;
  font-size:11px;
  text-transform:uppercase;
  letter-spacing:.6px;
}
.phaseChip strong{
  display:block;
  margin-top:7px;
  font-size:20px;
  font-family:"Consolas","Lucida Console",monospace;
}
.meta{
  margin-top:12px;
  padding-top:12px;
  border-top:1px solid #ffffff22;
  display:grid;
  gap:6px;
}
.metaLine{
  font-size:12px;
  color:#d9eef6;
  word-break:break-word;
}
.metaLine.muted{color:#bdd7e2}
.footerMark{
  margin-top:16px;
  padding:10px 0 4px;
  text-align:center;
  color:#9dc0ce;
  font-size:12px;
  letter-spacing:1.4px;
  text-transform:uppercase;
}
@keyframes pulseIcon{
  0%,100%{transform:translateY(1px) scale(1)}
  50%{transform:translateY(1px) scale(1.08)}
}
@media(max-width:390px){
  .brand{font-size:24px}
  .heroGrid{grid-template-columns:1fr}
  .gaugeWrap{width:176px;height:176px}
  .currentValue{font-size:44px}
  .stats,.chartLegend,.phaseRow{gap:8px}
  .stat strong{font-size:22px}
}
@media(max-width:520px){
  .app{padding:14px 12px 18px}
}
</style>
<link rel="manifest" href="/manifest.json">
<meta name="theme-color" content="#102635">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="RotosisEVSE">
</head><body class="state-A">
<div class="app">
  <div class="top">
    <div>
      <div class="brand">RotosisEVSE</div>
      <div class="brandSub">Mobil ana ekran</div>
    </div>
    <div class="sync wait" id="sync">WAIT</div>
  </div>

  <section class="card hero">
    <div class="heroHead">
      <div class="chipRow">
        <span class="chip main" id="state">STATE:A</span>
        <span class="chip" id="relay">R:RESET</span>
      </div>
      <div class="stateVisual" id="stateIcon" aria-hidden="true">
        <svg class="stateGlyph" viewBox="0 0 64 64" aria-hidden="true">
          <circle class="stateGlyphRing" cx="32" cy="32" r="18"></circle>
          <path class="stateGlyphBolt" d="M36 12 L24 33 L31 33 L27 51 L42 28 L34 28 Z"></path>
        </svg>
      </div>
    </div>

    <div class="heroGrid">
      <div class="heroCopy">
        <div class="eyebrow">Durum</div>
        <div class="stateName" id="stateName">Haz&#305;r</div>
        <div class="stateHint" id="stateHint">Ara&#231; bekleniyor</div>

        <div class="heroMiniRow">
          <div class="heroMini">
            <span>Hat dolulu&#287;u</span>
            <strong id="loadPct">0%</strong>
          </div>
          <div class="heroMini">
            <span>Ak&#305;m limiti</span>
            <strong><span id="limitA">32.0</span> A</strong>
          </div>
        </div>
      </div>

      <div class="gaugeWrap">
        <div class="gaugeShell"></div>
        <svg class="gaugeSvg" viewBox="0 0 220 220" aria-hidden="true">
          <defs>
            <linearGradient id="gaugeGradient" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stop-color="#72d7ff"></stop>
              <stop offset="55%" stop-color="var(--accent)"></stop>
              <stop offset="100%" stop-color="#d7fff2"></stop>
            </linearGradient>
          </defs>
          <circle class="gaugeTrack" cx="110" cy="110" r="72"></circle>
          <circle class="gaugeProgress" id="gaugeValue" cx="110" cy="110" r="72"></circle>
        </svg>
        <div class="gaugeCenter">
          <div class="currentLine">
            <span class="currentValue" id="it">0.0</span>
            <span class="currentUnit">A</span>
          </div>
          <div class="currentSub">Anl&#305;k aktar&#305;m</div>
          <div class="gaugeMeta">
            <span id="gaugeLabel">Hat dolulu&#287;u</span>
            <strong id="gaugePct">0%</strong>
          </div>
        </div>
      </div>
    </div>
  </section>

  <div class="actions">
    <button class="btn stop" id="chargeActionBtn" onclick="toggleChargeAction()"><span class="btnIcon" id="chargeActionIcon">&#9632;</span><span id="chargeActionText">&#350;arj&#305; Durdur</span></button>
    <a class="btn ghost" href="/admin"><span class="btnIcon">&#9881;</span><span>Ayarlar</span></a>
    <button class="btn ghost install" id="installBtn"><span class="btnIcon">&#8681;</span><span>Uygulama Kur</span></button>
  </div>

  <div class="stats">
    <div class="stat">
      <div class="statLabel"><i class="statIcon">&#9889;</i><span>G&#252;&#231;</span></div>
      <strong><span id="pwr">0.00</span> <em>kW</em></strong>
      <small>Anl&#305;k aktar&#305;m</small>
    </div>
    <div class="stat">
      <div class="statLabel"><i class="statIcon">&#128267;</i><span>Enerji</span></div>
      <strong><span id="ekwh">0.000</span> <em>kWh</em></strong>
      <small id="energyMeta">Aktif seans</small>
    </div>
    <div class="stat">
      <div class="statLabel"><i class="statIcon">&#9716;</i><span>S&#252;re</span></div>
      <strong id="tsec">00:00</strong>
      <small id="timeMeta">1 faz</small>
    </div>
    <div class="stat">
      <div class="statLabel"><i class="statIcon">&#9685;</i><span>Doluluk</span></div>
      <strong id="loadPctCard">0%</strong>
      <small id="limitMeta">Limit bazl&#305;</small>
    </div>
  </div>

  <section class="card carCard">
    <div class="carWrap">
      <div class="carFill" id="carFill"><div class="carFillInner" id="carFillInner"></div></div>
      <img id="carSvg" class="carSvg" alt="Car" src="data:image/png;base64,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" />
    </div>
    <div class="batteryRow">
      <span class="badgeIcon">&#9889;</span>
      <span>Batarya Seviyesi</span>
      <span id="battPct">% --</span>
    </div>
    <div class="chargeBar" id="chargeBar"><div class="chargeBarInner" id="chargeBarInner"></div></div>
  </section>

  <section class="card chartCard">
    <div class="sectionHead">
      <div>
        <div class="eyebrow">Canl&#305; Grafik</div>
        <div class="sectionTitle">Ak&#305;m ak&#305;&#351;&#305;</div>
      </div>
      <div class="sectionValue" id="chartNow">0.0 A</div>
    </div>

    <div class="chartFrame">
      <svg class="chartSvg" viewBox="0 0 320 160" preserveAspectRatio="none" aria-hidden="true">
        <defs>
          <linearGradient id="chartFill" x1="0%" y1="0%" x2="0%" y2="100%">
            <stop offset="0%" stop-color="#59f3ce66"></stop>
            <stop offset="100%" stop-color="#59f3ce00"></stop>
          </linearGradient>
        </defs>
        <line class="chartGrid" x1="0" y1="28" x2="320" y2="28"></line>
        <line class="chartGrid" x1="0" y1="80" x2="320" y2="80"></line>
        <line class="chartGrid" x1="0" y1="132" x2="320" y2="132"></line>
        <path class="chartArea" id="chartArea" d="M 12 146 L 12 146 L 308 146 Z"></path>
        <polyline class="chartLine" id="chartLine" points="12,146 308,146"></polyline>
        <circle class="chartDot" id="chartDot" cx="308" cy="146" r="6"></circle>
      </svg>
    </div>

    <div class="chartLegend">
      <span>Tepe<strong id="chartPeak">0.0 A</strong></span>
      <span>Ortalama<strong id="chartAvg">0.0 A</strong></span>
      <span>Pencere<strong id="chartWindow">0 sn</strong></span>
    </div>
  </section>

  <section class="card statusCard ok" id="alarmBox">
    <div class="statusHead">
      <div class="statusBadge">
        <span class="statusOrb" id="statusOrb"></span>
        <span id="statusCode">Durum A</span>
      </div>
      <div class="statusMeta" id="alarmMeta">Takip devam ediyor</div>
    </div>
    <div class="statusText" id="alarmText">Sistem normal</div>

    <div class="phaseRow">
      <div class="phaseChip"><span>Faz 1</span><strong id="i1">0.0 A</strong></div>
      <div class="phaseChip"><span>Faz 2</span><strong id="i2">0.0 A</strong></div>
      <div class="phaseChip"><span>Faz 3</span><strong id="i3">0.0 A</strong></div>
    </div>

    <div class="meta">
      <div class="metaLine" id="wifiLine">Wi-Fi: -</div>
      <div class="metaLine"><span id="host">-</span> | <span id="ip">-</span></div>
      <div class="metaLine muted" id="ts">-</div>
    </div>
  </section>
</div>
<script>
let deferredPrompt=null;
const POLL_MS=1200;
const MAX_POINTS=40;
let livePoints=[];
let chartCeil=32;
let currentChargeModeId=0;
const stateMeta={
  A:{name:"Haz\u0131r",hint:"Ara\u00e7 bekleniyor"},
  B:{name:"Ba\u011fl\u0131",hint:"Ara\u00e7 ba\u011fland\u0131"},
  C:{name:"\u015earjda",hint:"Enerji aktar\u0131l\u0131yor"},
  D:{name:"Havaland\u0131rma",hint:"\u015earj s\u00fcr\u00fcyor"},
  E:{name:"Hata",hint:"Pilot hata durumu"},
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
  if(carEls.chargeBar) carEls.chargeBar.classList.toggle("charging",charging);
  if(carEls.carSvg) carEls.carSvg.classList.toggle("charging",charging);

  const socVal=Number.isFinite(d.soc)?d.soc:(Number.isFinite(d.batt)?d.batt:null);
  let pctRaw=Number.isFinite(socVal)?socVal:Math.round((Number(d.eKWh)||0)/BAT_KWH*100);
  pctRaw=Math.max(0,Math.min(100,pctRaw));
  const pct=charging?Math.max(pctRaw,8):pctRaw;
  let fillPct=pct;
  if(isCable||isError) fillPct=100;
  if(st==="A") fillPct=0;
  const scale=Math.max(0.05,Math.min(1,(fillPct||0)/100));
  carEls.carFillInner.style.height=(scale*100)+"%";
  if(carEls.chargeBarInner){
    carEls.chargeBarInner.style.setProperty("--pct",scale);
    carEls.chargeBarInner.style.transform="scaleX("+scale+")";
    carEls.chargeBarInner.style.animation=charging?"barMove 1.6s linear infinite":"none";
  }
  carEls.battPct.textContent="% "+(Number.isFinite(fillPct)?fillPct:"--");
}
function fmtTime(sec){
  const h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60),s=sec%60;
  if(h>0) return String(h).padStart(2,"0")+":"+String(m).padStart(2,"0")+":"+String(s).padStart(2,"0");
  return String(m).padStart(2,"0")+":"+String(s).padStart(2,"0");
}
function clamp(v,min,max){return Math.max(min,Math.min(max,v))}
function setText(id,value){
  const el=document.getElementById(id);
  if(el) el.textContent=value;
}
function chargeCmd(mode){fetch('/charge_cmd?m='+mode).then(()=>pull()).catch(()=>{});}
function updateChargeAction(modeId){
  currentChargeModeId=modeId;
  const btn=document.getElementById("chargeActionBtn");
  const icon=document.getElementById("chargeActionIcon");
  const text=document.getElementById("chargeActionText");
  if(!btn||!icon||!text) return;
  if(modeId===2){
    btn.className="btn ok";
    icon.innerHTML="&#9654;";
    text.textContent="\u015earj\u0131 S\u00fcrd\u00fcr";
  }else{
    btn.className="btn stop";
    icon.innerHTML="&#9632;";
    text.textContent="\u015earj\u0131 Durdur";
  }
}
function toggleChargeAction(){
  chargeCmd(currentChargeModeId===2?0:2);
}
function setSync(ok){
  const el=document.getElementById("sync");
  if(!el) return;
  el.textContent=ok?"LIVE":"WAIT";
  el.className=ok?"sync":"sync wait";
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
  if(box) box.className="card statusCard "+(level===2?"err":(level===1?"warn":"ok"));
  const orb=document.getElementById("statusOrb");
  if(orb) orb.className="statusOrb"+(level===2?" err":(level===1?" warn":""));
  setText("alarmText",text||"Sistem normal");
  setText("alarmMeta","Durum kodu "+state+" | "+(staOk?"Wi-Fi ba\u011fl\u0131":"Wi-Fi yok"));
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
    chartCeil=Math.max(6,totalLimit);

    setText('it',it.toFixed(1));
    setText('i1',ia.toFixed(1)+" A");
    setText('i2',ib.toFixed(1)+" A");
    setText('i3',ic.toFixed(1)+" A");
    setText('pwr',powerKw.toFixed(2));
    setText('ekwh',energy.toFixed(3));
    setText('tsec',fmtTime(timeSec));
    setText('energyMeta',liveSession?"Aktif seans":"Son okuma");
    setText('timeMeta',phaseCount+" faz");
    setText('limitA',limitA.toFixed(1));
    setText('limitMeta',phaseCount+" faz / "+limitA.toFixed(1)+" A limit");
    setText('gaugeLabel',phaseCount+" faz / "+limitA.toFixed(1)+" A");
    updateChargeAction(Number(d.modeId)||0);
    if(d.rLbl!==undefined) setText('relay',"R:"+d.rLbl);
    if(d.state!==undefined) updateState(d.state);
    if(d.ip!==undefined) setText('ip',d.ip);
    if(d.host!==undefined) setText('host',d.host);
    setGauge(loadPct);
    setText('ts',"Son g\u00fcncelleme: "+new Date().toLocaleTimeString("tr-TR",{hour:"2-digit",minute:"2-digit",second:"2-digit"}));
    const wifiText=(d.wifiSsid&&d.wifiSsid!=="-") ? ("Wi-Fi: "+d.wifiSsid+((d.wifiLoc&&d.wifiLoc!=="-")?" / "+d.wifiLoc:"")) : "Wi-Fi: ba\u011fl\u0131 de\u011fil";
    setText('wifiLine',wifiText);
    renderAlarm(d.alarmLv||0, d.alarmTxt||"Sistem normal", d.state||"A", !!d.staOk);
    updateCar(d);
    pushLivePoint(it);
    setSync(true);
  }).catch(()=>{ setSync(false); });
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
setInterval(pull,POLL_MS);
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

</style></head><body>

<div class="wrap">
  <div class="card">
    <h2>CANLI VERÄ°LER</h2>
    <div class="kv"><div class="k">State (Stable/Raw)</div><div class="v mono"><span id="sStb">-</span> / <span id="sRaw">-</span></div></div>
    <div class="kv"><div class="k">CP High / Low</div><div class="v mono"><span id="cH">-</span> / <span id="cL">-</span></div></div>
    <div class="kv"><div class="k">ADC High / Low</div><div class="v mono"><span id="aH">-</span> / <span id="aL">-</span></div></div>

    <div class="sep"></div>
    <h2>Zamanlama ve HÄ±z AyarlarÄ±</h2>
    <div class="kv"><div class="k">Loop DÃ¶ngÃ¼sÃ¼ (ms)</div><div class="v"><input id="lInt" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">RÃ¶le AÃ§ma (ms)</div><div class="v"><input id="onD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">RÃ¶le BÄ±rakma (ms)</div><div class="v"><input id="offD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Stable Count</div><div class="v"><input id="stb" onfocus="p()" onblur="r()"></div></div>

    <div class="sep"></div>
    <h2>EÅŸikler / Kalibrasyon</h2>
    <div class="kv"><div class="k">Divider</div><div class="v"><input id="div" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">TH_B / TH_C</div><div class="v" style="display:flex;gap:5px"><input id="thb" onfocus="p()" onblur="r()"> <input id="thc" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">TH_D / TH_E</div><div class="v" style="display:flex;gap:5px"><input id="thd" onfocus="p()" onblur="r()"> <input id="the" onfocus="p()" onblur="r()"></div></div>

    <div class="btns" style="margin-top:10px">
      <button class="primary" onclick="applyCal()">UYGULA</button>
      <button onclick="pull(true)">YENÄ°LE</button>
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
    <div class="kv"><div class="k">Durum</div><div class="v mono"><span id="otaStatus">-</span></div></div>
    <div class="kv"><div class="k">Son Kontrol</div><div class="v mono"><span id="otaAge">-</span></div></div>
    <div class="kv"><div class="k">Hata</div><div class="v mono"><span id="otaError">Hata yok</span></div></div>
    <div class="btns" style="margin-top:10px">
      <button class="primary" onclick="runOtaCheckAdmin()">OTA SIMDI KONTROL ET</button>
    </div>



  </div>
</div>


<script>
let paused = false; let t;
const inps = ["lInt","onD","offD","stb","div","thb","thc","thd","the"];
const keys = ["lInt","onD","offD","stable","div","thb","thc","thd","the"];

function p(){ paused=true; clearTimeout(t); }
function r(){ t=setTimeout(()=>{paused=false;},3000); }
function send(path){ fetch(path).then(_=>pull(false)); }

function fmtTime(sec){
  const m = Math.floor(sec/60);
  const s = sec%60;
  const mm = String(m).padStart(2, "0");
  const ss = String(s).padStart(2, "0");
  return mm + ":" + ss;
}

function fmtOtaAge(ms){
  const sec = Math.max(0, Math.round((Number(ms) || 0) / 1000));
  if (sec === 0) return "hemen simdi";
  if (sec < 60) return sec + " sn once";
  const min = Math.floor(sec / 60);
  const rem = sec % 60;
  return min + " dk " + rem + " sn once";
}

function runOtaCheckAdmin(){
  fetch('/ota_check', {cache:'no-store'}).then(_ => {
    document.getElementById('otaStatus').textContent = 'check_requested';
  });
}

function pull(force=false){
  if(paused && !force) return;
  fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{
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
    if(d.otaCur !== undefined) document.getElementById('otaCurVer').textContent = d.otaCur;
    if(d.otaRemote !== undefined) document.getElementById('otaRemoteVer').textContent = d.otaRemote;
    if(d.otaStatus !== undefined) document.getElementById('otaStatus').textContent = d.otaStatus;
    if(d.otaAgeMs !== undefined) document.getElementById('otaAge').textContent = fmtOtaAge(d.otaAgeMs);
    if(d.otaErr !== undefined) document.getElementById('otaError').textContent = d.otaErr && d.otaErr.length ? d.otaErr : 'Hata yok';

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
  });
}

function applyCal(){
  const q = new URLSearchParams();
  inps.forEach((id, i) => q.append(keys[i]==="stable"?"s":keys[i], document.getElementById(id).value));
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
static void handleRoot() {
  noteWebActivity();
  noteHttpResponseSent();
  server.send_P(200, "text/html", USER_HTML);
}
static void handleAdmin() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  noteHttpResponseSent();
  server.send_P(200, "text/html", MAIN_HTML);
}
static void handlePing() { noteWebActivity(); noteHttpResponseSent(); server.send(200, "text/plain", "OK"); }
static void handleManifest() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
static void handleServiceWorker() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/javascript", SERVICE_WORKER_JS); }
static void handleAppIcon() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "image/svg+xml", APP_ICON_SVG); }
static void handleOtaCheck() {
  noteWebActivity();
  OTA_Manager::triggerCheckNow();
  noteHttpResponseSent();
  server.send(200, "application/json", "{\"ok\":1}");
}

// Status endpoint'i kullanici panelinin ana veri kaynagidir.
static void handleStatus() {
  noteWebActivity();
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

  char json[2304];
  snprintf(
    json, sizeof(json),
    "{\"lInt\":%d,\"onD\":%lu,\"offD\":%lu,\"stable\":%d,"
    "\"cpHigh\":%.2f,\"cpLow\":%.2f,\"adcHigh\":%.3f,\"adcLow\":%.3f,"
    "\"stateRaw\":\"%s\",\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,"
    "\"pW\":%.1f,\"eKWh\":%.3f,\"tSec\":%lu,\"phase\":%d,\"rLbl\":\"%s\","
    "\"wifiSsid\":\"%s\",\"wifiLoc\":\"%s\",\"ip\":\"%s\",\"host\":\"%s\","
    "\"state\":\"%s\",\"div\":%.3f,\"thb\":%.2f,\"thc\":%.2f,\"thd\":%.2f,\"the\":%.2f,"
    "\"modeId\":%d,\"mode\":\"%s\",\"limitA\":%.1f,\"staOk\":%d,"
    "\"otaCur\":\"%s\",\"otaRemote\":\"%s\",\"otaStatus\":\"%s\",\"otaErr\":\"%s\",\"otaAgeMs\":%lu,"
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
    g_chargeMode,
    chargeModeLabel(g_chargeMode),
    safeFinite(g_currentLimitA),
    staOk ? 1 : 0,
    otaCurrent,
    otaRemote,
    otaStatus,
    otaError,
    (unsigned long)otaAgeMs,
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

  server.send(200, "application/json", json);
  noteHttpResponseSent();
}

// Gecmis seanslar icin ayri JSON endpoint.
static void handleHistory() {
  noteWebActivity();
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
  noteHttpResponseSent();
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
  noteHttpResponseSent();
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
  noteHttpResponseSent();
}

// Web admin panelinden gelen CP / relay / timing ayarlari burada uygulanir.
static void handleCalibApply() {
  if (!requireAdminAuth()) return;
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
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handleRelay() {
  if (server.hasArg("on")) relay_set(server.arg("on") == "1");
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handleRelayAuto() {
  if (server.hasArg("en")) relay_set_auto_enabled(server.arg("en") == "1");
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handlePulseReset() {
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_RESET_PIN);
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handlePulseSet() {
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_SET_PIN);
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
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
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/sw.js", HTTP_GET, handleServiceWorker);
  server.on("/app-icon.svg", HTTP_GET, handleAppIcon);
  server.on("/ota_check", HTTP_GET, handleOtaCheck);
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
  ElegantOTA.begin(&server, kAdminUser, kAdminPassword);
  server.begin();
}

void web_loop() {
  // Arka plan servisleri her loop'ta buradan yurutulur.
  ArduinoOTA.handle();
  server.handleClient();
  ElegantOTA.loop();

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
  if (!staOk) return false;
  if (s_successfulHttpResponses == 0) return false;
  return s_lastHttpRequestMs != 0;
}

