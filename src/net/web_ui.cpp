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



  </div>
</div>
<div class="footerMark">turgay</div>


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

function pull(force=false){
  if(paused && !force) return;
  fetch('/status').then(r=>r.json()).then(d=>{
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
static void handleRoot() { server.send_P(200, "text/html", USER_HTML); }
static void handleAdmin() {
  if (!requireAdminAuth()) return;
  server.send_P(200, "text/html", MAIN_HTML);
}
static void handlePing() { server.send(200, "text/plain", "OK"); }
static void handleManifest() { server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
static void handleServiceWorker() { server.send_P(200, "application/javascript", SERVICE_WORKER_JS); }
static void handleAppIcon() { server.send_P(200, "image/svg+xml", APP_ICON_SVG); }

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
  float iMax = ia;
  if (ib > iMax) iMax = ib;
  if (ic > iMax) iMax = ic;

  String wifiSsid = "-";
  String wifiLoc = "-";
  String ipStr = "-";
  String hostStr = String(kHostName) + ".local";
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
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

  char json[1440];
  snprintf(
    json, sizeof(json),
    "{\"lInt\":%d,\"onD\":%lu,\"offD\":%lu,\"stable\":%d,"
    "\"cpHigh\":%.2f,\"cpLow\":%.2f,\"adcHigh\":%.3f,\"adcLow\":%.3f,"
    "\"stateRaw\":\"%s\",\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,"
    "\"pW\":%.1f,\"eKWh\":%.3f,\"tSec\":%lu,\"phase\":%d,\"rLbl\":\"%s\","
    "\"wifiSsid\":\"%s\",\"wifiLoc\":\"%s\",\"ip\":\"%s\",\"host\":\"%s\","
    "\"state\":\"%s\",\"div\":%.3f,\"thb\":%.2f,\"thc\":%.2f,\"thd\":%.2f,\"the\":%.2f,"
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

  server.send(200, "application/json", json);
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
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/sw.js", HTTP_GET, handleServiceWorker);
  server.on("/app-icon.svg", HTTP_GET, handleAppIcon);
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

