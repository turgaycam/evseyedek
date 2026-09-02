#include "web_ui.h"
#include <WiFi.h>
#include <WiFiMulti.h>

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>
#include <math.h>

#include "app_config.h"
#include "app_pins.h"
#include "OTA_Manager.h"
#include "vehicle_top_art_svg.h"
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
extern float g_targetCurrentLimitA;
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
static const char* kHostNameBase = EVSE_HOSTNAME;
static const char* kStationName = "Rotosis Robotlu Otomasyon";
static const char* kDefaultStationAddress = "Fevzi Cakmak Mah. Sehit Ibrahim Betin Cd. No:4/F, Arli Sanayi Sitesi, Karatay / Konya";
static constexpr double kDefaultMapLat = 37.94559;
static constexpr double kDefaultMapLng = 32.58082;
static constexpr uint16_t kMapRadiusM = 520;
static constexpr size_t kMaxUserScreenCssLen = 4096;
static char s_deviceMac[18] = "";
static char s_stationCode[7] = "";
static char s_stationLabel[32] = "Istasyon";
static char s_stationCustomLabel[32] = "";
static bool s_stationLabelCustom = false;
static char s_hostName[32] = EVSE_HOSTNAME;
static char s_stationAddress[160] = "";
static double s_mapLat = kDefaultMapLat;
static double s_mapLng = kDefaultMapLng;
static bool s_customWifiEnabled = false;
static bool s_wifiUseDhcp = true;
static String s_customWifiSsid;
static String s_customWifiPassword;
static String s_userScreenCustomCss;
static IPAddress s_staticIp(192, 168, 1, 200);
static IPAddress s_staticGateway(192, 168, 1, 1);
static IPAddress s_staticSubnet(255, 255, 255, 0);
static IPAddress s_staticDns1(8, 8, 8, 8);
static IPAddress s_staticDns2(1, 1, 1, 1);
static uint32_t s_lastWifiScanMs = 0;
static String s_lastWifiScanJson = "{\"items\":[]}";

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
static char s_jsonBuf[4096];
static bool s_mdnsEnabled = false;
static TaskHandle_t s_webTaskHandle = nullptr;
static bool s_serverStarted = false;
static uint32_t s_resetTotalCount = 0;
static uint32_t s_resetNowCount = 0;
static uint32_t s_resetHistoryCount = 0;
static uint32_t s_resetLastSec = 0;
static uint8_t s_resetLastModeId = 0;
static Preferences s_resetPrefs;
static bool s_resetPrefsReady = false;
static void web_task_runner(void* arg);
static void refreshDeviceIdentity();
static const char* currentHostName();

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

static bool isValidLatitude(double value) {
  return value >= -90.0 && value <= 90.0;
}

static bool isValidLongitude(double value) {
  return value >= -180.0 && value <= 180.0;
}

static bool nearlyEqualCoord(double a, double b) {
  return fabs(a - b) < 0.00001;
}

static String ipToString(const IPAddress& ip) {
  return ip.toString();
}

static bool parseIpAddressArg(const String& value, IPAddress& out) {
  int parts[4] = {0, 0, 0, 0};
  int part = 0;
  int start = 0;
  String src = value;
  src.trim();
  if (src.length() == 0) return false;

  for (int i = 0; i <= src.length(); ++i) {
    if (i == src.length() || src[i] == '.') {
      if (part >= 4 || i == start) return false;
      int v = src.substring(start, i).toInt();
      if (v < 0 || v > 255) return false;
      parts[part++] = v;
      start = i + 1;
    }
  }
  if (part != 4) return false;
  out = IPAddress(parts[0], parts[1], parts[2], parts[3]);
  return true;
}

static void rebuildStationAddress() {
  if (nearlyEqualCoord(s_mapLat, kDefaultMapLat) && nearlyEqualCoord(s_mapLng, kDefaultMapLng)) {
    snprintf(s_stationAddress, sizeof(s_stationAddress), "%s", kDefaultStationAddress);
    return;
  }

  snprintf(
    s_stationAddress,
    sizeof(s_stationAddress),
    "Secilen konum: %.5f, %.5f",
    s_mapLat,
    s_mapLng
  );
}

static bool fetchReverseGeocodedAddress(double lat, double lng, char* out, size_t outSize) {
  if (out == nullptr || outSize == 0) return false;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(5000);

  String url = String("https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=18&addressdetails=1&lat=") +
               String(lat, 6) + "&lon=" + String(lng, 6);
  if (!http.begin(client, url)) return false;

  http.addHeader("User-Agent", "RotosisEVSE/1.0");
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  StaticJsonDocument<1536> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) return false;

  const char* displayName = doc["display_name"] | "";
  if (!displayName[0]) return false;

  snprintf(out, outSize, "%s", displayName);
  return true;
}

static void refreshStationAddress(bool tryReverseGeocode) {
  if (tryReverseGeocode) {
    char resolved[160] = "";
    if (fetchReverseGeocodedAddress(s_mapLat, s_mapLng, resolved, sizeof(resolved))) {
      snprintf(s_stationAddress, sizeof(s_stationAddress), "%s", resolved);
      return;
    }
  }
  rebuildStationAddress();
}

static void loadLocationSettings() {
  if (s_resetPrefsReady) {
    double storedLat = s_resetPrefs.getDouble("mapLat", kDefaultMapLat);
    double storedLng = s_resetPrefs.getDouble("mapLng", kDefaultMapLng);
    String storedAddr = s_resetPrefs.getString("mapAddr", "");
    String storedLabel = s_resetPrefs.getString("stationLabel", "");
    storedLabel.trim();
    if (storedLabel.length() > 0) {
      s_stationLabelCustom = true;
      snprintf(s_stationCustomLabel, sizeof(s_stationCustomLabel), "%s", storedLabel.c_str());
    } else {
      s_stationLabelCustom = false;
      s_stationCustomLabel[0] = '\0';
    }
    refreshDeviceIdentity();
    if (isValidLatitude(storedLat) && isValidLongitude(storedLng)) {
      s_mapLat = storedLat;
      s_mapLng = storedLng;
      if (storedAddr.length() > 0) {
        snprintf(s_stationAddress, sizeof(s_stationAddress), "%s", storedAddr.c_str());
        return;
      }
    } else {
      s_mapLat = kDefaultMapLat;
      s_mapLng = kDefaultMapLng;
    }
  } else {
    s_mapLat = kDefaultMapLat;
    s_mapLng = kDefaultMapLng;
  }

  refreshStationAddress(false);
}

static void saveLocationSettings() {
  if (!s_resetPrefsReady) return;
  s_resetPrefs.putDouble("mapLat", s_mapLat);
  s_resetPrefs.putDouble("mapLng", s_mapLng);
  s_resetPrefs.putString("mapAddr", s_stationAddress);
  s_resetPrefs.putString("stationLabel", s_stationLabelCustom ? String(s_stationCustomLabel) : String(""));
}

static void loadWifiSettings() {
  if (!s_resetPrefsReady) return;
  s_customWifiEnabled = s_resetPrefs.getBool("wifi_en", false);
  s_wifiUseDhcp = s_resetPrefs.getBool("wifi_dhcp", true);
  s_customWifiSsid = s_resetPrefs.getString("wifi_ssid", "");
  s_customWifiPassword = s_resetPrefs.getString("wifi_pass", "");

  IPAddress parsed;
  if (parseIpAddressArg(s_resetPrefs.getString("wifi_ip", "192.168.1.200"), parsed)) s_staticIp = parsed;
  if (parseIpAddressArg(s_resetPrefs.getString("wifi_gw", "192.168.1.1"), parsed)) s_staticGateway = parsed;
  if (parseIpAddressArg(s_resetPrefs.getString("wifi_sub", "255.255.255.0"), parsed)) s_staticSubnet = parsed;
  if (parseIpAddressArg(s_resetPrefs.getString("wifi_d1", "8.8.8.8"), parsed)) s_staticDns1 = parsed;
  if (parseIpAddressArg(s_resetPrefs.getString("wifi_d2", "1.1.1.1"), parsed)) s_staticDns2 = parsed;
}

static void saveWifiSettings() {
  if (!s_resetPrefsReady) return;
  s_resetPrefs.putBool("wifi_en", s_customWifiEnabled);
  s_resetPrefs.putBool("wifi_dhcp", s_wifiUseDhcp);
  s_resetPrefs.putString("wifi_ssid", s_customWifiSsid);
  s_resetPrefs.putString("wifi_pass", s_customWifiPassword);
  s_resetPrefs.putString("wifi_ip", ipToString(s_staticIp));
  s_resetPrefs.putString("wifi_gw", ipToString(s_staticGateway));
  s_resetPrefs.putString("wifi_sub", ipToString(s_staticSubnet));
  s_resetPrefs.putString("wifi_d1", ipToString(s_staticDns1));
  s_resetPrefs.putString("wifi_d2", ipToString(s_staticDns2));
}

static void rebuildKnownWifiList() {
  wifiMulti = WiFiMulti();
  for (size_t i = 0; i < (sizeof(kKnownWifis) / sizeof(kKnownWifis[0])); i++) {
    if (!hasText(kKnownWifis[i].ssid)) continue;
    wifiMulti.addAP(kKnownWifis[i].ssid, kKnownWifis[i].password);
  }
}

static void applyIpMode() {
  if (s_customWifiEnabled && !s_wifiUseDhcp) {
    WiFi.config(s_staticIp, s_staticGateway, s_staticSubnet, s_staticDns1, s_staticDns2);
  } else {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }
}

static bool connectConfiguredWifi(uint32_t timeoutMs) {
  if (!s_customWifiEnabled || s_customWifiSsid.length() == 0) return false;
  applyIpMode();
  WiFi.begin(s_customWifiSsid.c_str(), s_customWifiPassword.c_str());
  uint32_t start = millis();
  while ((millis() - start) < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0) {
      return true;
    }
    delay(100);
  }
  return false;
}

static void reconnectWifiNow() {
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  refreshDeviceIdentity();
  WiFi.setHostname(currentHostName());
  applyIpMode();
  rebuildKnownWifiList();

  if (s_customWifiEnabled && s_customWifiSsid.length() > 0) {
    WiFi.begin(s_customWifiSsid.c_str(), s_customWifiPassword.c_str());
  }
}

static void loadCurrentLimitSetting() {
  if (s_resetPrefsReady) {
    float stored = s_resetPrefs.getFloat("limitA", 32.0f);
    if (stored < 6.0f) stored = 6.0f;
    if (stored > 32.0f) stored = 32.0f;
    g_targetCurrentLimitA = stored;
  } else {
    g_targetCurrentLimitA = 32.0f;
  }
}

static void saveCurrentLimitSetting() {
  if (!s_resetPrefsReady) return;
  s_resetPrefs.putFloat("limitA", g_targetCurrentLimitA);
}

static void loadUserScreenCssSetting() {
  if (!s_resetPrefsReady) {
    s_userScreenCustomCss = "";
    return;
  }
  s_userScreenCustomCss = s_resetPrefs.getString("user_css", "");
  if (s_userScreenCustomCss.length() > kMaxUserScreenCssLen) {
    s_userScreenCustomCss.remove(kMaxUserScreenCssLen);
  }
}

static void saveUserScreenCssSetting() {
  if (!s_resetPrefsReady) return;
  s_resetPrefs.putString("user_css", s_userScreenCustomCss);
}

static void refreshDeviceIdentity() {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);

  snprintf(
    s_deviceMac, sizeof(s_deviceMac),
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
  snprintf(
    s_stationCode, sizeof(s_stationCode),
    "%02X%02X%02X",
    mac[3], mac[4], mac[5]
  );
  snprintf(
    s_stationLabel, sizeof(s_stationLabel),
    "Istasyon %s",
    s_stationCode
  );
  if (s_stationLabelCustom && s_stationCustomLabel[0]) {
    snprintf(s_stationLabel, sizeof(s_stationLabel), "%s", s_stationCustomLabel);
  }
  snprintf(
    s_hostName, sizeof(s_hostName),
    "%s-%s",
    kHostNameBase,
    s_stationCode
  );
}

static const char* currentHostName() {
  if (!s_hostName[0]) refreshDeviceIdentity();
  return s_hostName;
}

static void resetManualOtaState() {
  s_manualOta.active = false;
  s_manualOta.updateBegun = false;
  s_manualOta.success = false;
  s_manualOta.uploadedName = "";
  s_manualOta.lastError = "";
}

static void refreshMdns() {
  if (!s_mdnsEnabled) return;
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
    if (!MDNS.begin(currentHostName())) {
      Serial.println("[mDNS] Start failed");
      return;
    }
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    Serial.print("[mDNS] Ready: http://");
    Serial.print(currentHostName());
    Serial.println(".local");
  }
}

static void ensureServerStarted() {
  if (s_serverStarted) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;
  Serial.println("[WEB] server.begin() delayed start");
  server.begin();
  s_serverStarted = true;
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

static String htmlEscape(const String& value) {
  String out;
  out.reserve(value.length() + 16);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

static String styleEscape(const String& value) {
  String out = value;
  out.replace("</style", "<\\/style");
  out.replace("</STYLE", "<\\/STYLE");
  return out;
}

static String jsonEscape(const String& value) {
  String out = value;
  out.replace("\\", "\\\\");
  out.replace("\"", "\\\"");
  out.replace("\n", "\\n");
  out.replace("\r", "");
  return out;
}

struct DisplayCurrentState {
  bool primed = false;
  float ia = 0.0f;
  float ib = 0.0f;
  float ic = 0.0f;
};

static DisplayCurrentState s_displayCurrent;

static float smoothDisplayCurrent(float previous, float target) {
  float delta = target - previous;
  float alpha = (fabsf(delta) > 2.0f) ? 0.28f : 0.16f;
  if (target < previous) alpha *= 0.72f;
  float blended = previous + (delta * alpha);
  if (blended < 0.08f) blended = 0.0f;
  return blended;
}

static void harmonizeThreePhaseDisplay(float* ia, float* ib, float* ic) {
  if (!ia || !ib || !ic) return;
  const float activeTh = 0.90f;
  if (*ia < activeTh || *ib < activeTh || *ic < activeTh) return;

  float maxV = *ia;
  if (*ib > maxV) maxV = *ib;
  if (*ic > maxV) maxV = *ic;

  float minV = *ia;
  if (*ib < minV) minV = *ib;
  if (*ic < minV) minV = *ic;

  float avg = (*ia + *ib + *ic) / 3.0f;
  float spread = maxV - minV;
  if (spread <= 0.45f || (avg > 0.1f && (spread / avg) <= 0.06f)) {
    *ia = avg;
    *ib = avg;
    *ic = avg;
  }
}

static void updateDisplayCurrents(float rawIa,
                                  float rawIb,
                                  float rawIc,
                                  float* outIa,
                                  float* outIb,
                                  float* outIc) {
  if (!s_displayCurrent.primed) {
    s_displayCurrent.ia = rawIa;
    s_displayCurrent.ib = rawIb;
    s_displayCurrent.ic = rawIc;
    s_displayCurrent.primed = true;
  } else {
    s_displayCurrent.ia = smoothDisplayCurrent(s_displayCurrent.ia, rawIa);
    s_displayCurrent.ib = smoothDisplayCurrent(s_displayCurrent.ib, rawIb);
    s_displayCurrent.ic = smoothDisplayCurrent(s_displayCurrent.ic, rawIc);
  }

  harmonizeThreePhaseDisplay(&s_displayCurrent.ia, &s_displayCurrent.ib, &s_displayCurrent.ic);

  if (outIa) *outIa = roundf(s_displayCurrent.ia * 10.0f) / 10.0f;
  if (outIb) *outIb = roundf(s_displayCurrent.ib * 10.0f) / 10.0f;
  if (outIc) *outIc = roundf(s_displayCurrent.ic * 10.0f) / 10.0f;
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
:root{--bg:#06121d;--bg2:#0a2233;--panel:rgba(8,24,37,.78);--panelStrong:rgba(12,31,47,.92);--line:rgba(72,209,164,.18);--text:#ecfbf6;--muted:#8daea4;--accent:#37d8a2;--accentDeep:#15b987;--accentSoft:rgba(55,216,162,.18);--warn:#b6ef72;--err:#ff7d7d;--shadow:0 24px 50px rgba(1,9,16,.46);--shadowSoft:0 14px 32px rgba(2,10,18,.34);}
html,body{margin:0;padding:0}
body{position:relative;overflow-x:hidden;font-family:"Avenir Next","Segoe UI","Trebuchet MS",sans-serif;color:var(--text);min-height:100vh;background:
radial-gradient(circle at 15% 15%, rgba(0,255,255,.15) 0%, rgba(0,255,255,0) 35%),
radial-gradient(circle at 85% 85%, rgba(138,43,226,.15) 0%, rgba(138,43,226,0) 35%),
radial-gradient(circle at 50% 50%, rgba(255,215,0,.1) 0%, rgba(255,215,0,0) 50%),
linear-gradient(180deg,#0d0d0d 0%,#1a1a2e 40%,#16213e 80%,#0f0f23 100%);}
body::after{content:"";position:fixed;z-index:0;border-radius:999px;pointer-events:none;filter:blur(18px);}
body::after{width:290px;height:290px;right:-106px;bottom:92px;background:radial-gradient(circle, rgba(0,255,255,.2) 0%, rgba(0,255,255,0) 70%);animation:floatGlowB 18s ease-in-out infinite;}
body.state-A{--accent:#66c7ff;--accentDeep:#249ff0;--accentSoft:rgba(102,199,255,.18)}
body.state-B{--accent:#7ae6c4;--accentDeep:#2fc79a;--accentSoft:rgba(122,230,196,.18)}
body.state-C,body.state-D{--accent:#37d8a2;--accentDeep:#15b987;--accentSoft:rgba(55,216,162,.18)}
body.state-E,body.state-F{--accent:#ff8b8b;--accentDeep:#ff6464;--accentSoft:rgba(255,139,139,.18)}
.screenShell{position:relative;z-index:1;max-width:420px;margin:0 auto;padding:14px 12px 22px;}
.app{position:relative;z-index:1;max-width:396px;min-height:calc(100vh - 44px);margin:0 auto;padding:0 0 18px;animation:screenEnter .82s cubic-bezier(.22,1,.36,1);}
.phaseMeta{margin-top:16px;display:flex;justify-content:space-between;gap:10px;font-size:14px;color:#8bb5a6;}
.phaseMeta span{flex:1;padding:10px 12px;border-radius:16px;background:linear-gradient(180deg,rgba(12,32,50,.58),rgba(8,22,36,.38));border:1px solid rgba(151,210,255,.08);text-align:center}
.topbar{display:grid;grid-template-columns:44px 1fr 52px;align-items:center;gap:10px;}
.backBtn{width:44px;height:44px;border-radius:16px;display:grid;place-items:center;text-decoration:none;color:#dff9f0;font-size:30px;line-height:1;background:linear-gradient(180deg,rgba(18,42,61,.92),rgba(9,22,35,.88));border:1px solid rgba(79,225,178,.16);box-shadow:var(--shadowSoft);}
.statusWrap{display:flex;justify-content:center;}
.statusPill{display:inline-flex;align-items:center;gap:10px;padding:11px 16px;border-radius:999px;background:linear-gradient(135deg,rgba(13,31,46,.96),rgba(7,20,31,.9));border:2px solid color-mix(in srgb,var(--accentDeep) 52%, white);box-shadow:0 16px 30px rgba(2,10,18,.28);font-weight:800;font-size:15px;letter-spacing:.01em;color:#eefcf8;animation:fadeLift .82s .04s both;}
.statusIcon{width:24px;height:24px;display:grid;place-items:center;color:var(--accent);}
.statusIcon svg{width:24px;height:24px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round;}
.vehicleName{font-weight:800;}
.pillDash{opacity:.55}
.sync{justify-self:end;min-width:52px;height:36px;padding:0 10px;border-radius:999px;background:linear-gradient(180deg,rgba(15,35,54,.94),rgba(8,19,30,.9));border:1px solid rgba(87,209,171,.16);display:flex;align-items:center;justify-content:center;gap:7px;font-size:11px;font-weight:800;letter-spacing:.08em;color:#8fbbb0;text-transform:uppercase;box-shadow:var(--shadowSoft);}
.sync::before{content:"";width:8px;height:8px;border-radius:999px;background:#3e6f84;box-shadow:0 0 0 4px rgba(62,111,132,.18);}
.sync.live{color:#dffff4;border-color:rgba(55,216,162,.26);background:linear-gradient(180deg,rgba(18,49,58,.96),rgba(9,26,35,.94));}
.sync.live::before{background:var(--accent);box-shadow:0 0 0 4px color-mix(in srgb,var(--accent) 18%, transparent);animation:livePulse 1.8s ease-in-out infinite;}
.headline{display:none}
.dateLine{font-size:19px;font-weight:700;letter-spacing:.01em;color:#bde9ff;}
.locationLine{margin-top:8px;font-size:18px;line-height:1.35;font-weight:700;color:#f1fffb;}
.hintLine{margin-top:6px;font-size:14px;color:#84c8b5;}
.metricCard{max-width:338px;margin:20px auto 0;padding:20px 22px;border-radius:26px;background:linear-gradient(180deg,rgba(10,28,43,.82),rgba(7,19,31,.62));border:1px solid rgba(151,210,255,.14);box-shadow:0 24px 40px rgba(2,10,18,.28);backdrop-filter:blur(20px);animation:fadeLift 1s .26s both;}
.metricCard--stats{max-width:338px}
.metricCardArt{display:flex;justify-content:center;align-items:center;margin:0 0 14px;padding:6px 0 2px;color:rgba(208,237,255,.92)}
.vehicleStage{position:relative;width:min(78%,228px);isolation:isolate}
.vehicleStage img{display:block;width:100%;height:auto;opacity:.95;transform:rotate(90deg);transform-origin:50% 50%;filter:drop-shadow(0 12px 24px rgba(4,16,28,.22))}
.chargeOverlay{position:absolute;inset:0;pointer-events:none}
.chargeAura{position:absolute;left:19%;right:19%;top:26%;bottom:18%;border-radius:999px;background:
radial-gradient(circle at 50% 50%,rgba(124,247,215,.26) 0%,rgba(124,247,215,.13) 32%,rgba(124,247,215,0) 72%);
opacity:.18;transform:scale(.9);filter:blur(10px);transition:opacity .35s ease,transform .35s ease}
.chargeBeam{position:absolute;left:34%;right:34%;top:19%;bottom:15%;border-radius:999px;background:
linear-gradient(180deg,rgba(123,247,213,0) 0%,rgba(123,247,213,.12) 14%,rgba(207,255,245,.92) 34%,rgba(123,247,213,.18) 56%,rgba(123,247,213,0) 100%);
opacity:.2;filter:blur(5px);transform-origin:50% 50%;animation:chargeBeamFlow var(--chargeSpeed,2.6s) linear infinite}
.chargePulse{position:absolute;left:50%;width:13%;aspect-ratio:1;border-radius:999px;transform:translateX(-50%);background:
radial-gradient(circle,rgba(240,255,251,.98) 0%,rgba(136,245,210,.82) 38%,rgba(76,219,171,.08) 72%,rgba(76,219,171,0) 100%);
box-shadow:0 0 18px rgba(122,246,211,.42),0 0 34px rgba(122,246,211,.22);opacity:0;animation:chargeTravel var(--chargeSpeed,2.6s) linear infinite}
.chargePulse.p2{animation-delay:calc(var(--chargeSpeed,2.6s)/3)}
.chargePulse.p3{animation-delay:calc((var(--chargeSpeed,2.6s)/3)*2)}
.chargeSpark{position:absolute;left:50%;top:45%;width:20%;height:20%;transform:translate(-50%,-50%) rotate(-6deg);opacity:.18;filter:drop-shadow(0 0 12px rgba(142,248,219,.4));animation:sparkFlash 2.1s ease-in-out infinite}
.chargeSpark svg{display:block;width:100%;height:100%;fill:#dffff6}
.vehicleStage .chargeAura,.vehicleStage .chargeBeam,.vehicleStage .chargePulse,.vehicleStage .chargeSpark{animation-play-state:paused}
.vehicleStage.is-charging .chargeAura{opacity:var(--chargeOpacity,.72);transform:scale(1.04)}
.vehicleStage.is-charging .chargeBeam,.vehicleStage.is-charging .chargePulse,.vehicleStage.is-charging .chargeSpark{animation-play-state:running}
.vehicleStage.is-charging .chargeBeam{opacity:calc(.16 + (var(--chargeOpacity,.72)*.7))}
body.state-B .vehicleStage .chargeAura{background:radial-gradient(circle at 50% 50%,rgba(255,214,107,.24) 0%,rgba(255,214,107,.11) 34%,rgba(255,214,107,0) 72%)}
body.state-B .vehicleStage .chargeBeam{background:linear-gradient(180deg,rgba(255,214,107,0) 0%,rgba(255,214,107,.12) 14%,rgba(255,249,220,.92) 34%,rgba(255,214,107,.18) 56%,rgba(255,214,107,0) 100%)}
body.state-B .vehicleStage .chargePulse{background:radial-gradient(circle,rgba(255,252,241,.98) 0%,rgba(255,214,107,.84) 38%,rgba(255,214,107,.08) 72%,rgba(255,214,107,0) 100%);box-shadow:0 0 18px rgba(255,214,107,.4),0 0 34px rgba(255,214,107,.2)}
body.state-B .vehicleStage .chargeSpark svg{fill:#fff5cb}
body.state-C .vehicleStage .chargeAura,body.state-D .vehicleStage .chargeAura{background:radial-gradient(circle at 50% 50%,rgba(118,205,255,.26) 0%,rgba(118,205,255,.12) 34%,rgba(118,205,255,0) 72%)}
body.state-C .vehicleStage .chargeBeam,body.state-D .vehicleStage .chargeBeam{background:linear-gradient(180deg,rgba(118,205,255,0) 0%,rgba(118,205,255,.14) 14%,rgba(236,249,255,.94) 34%,rgba(118,205,255,.18) 56%,rgba(118,205,255,0) 100%)}
body.state-C .vehicleStage .chargePulse,body.state-D .vehicleStage .chargePulse{background:radial-gradient(circle,rgba(245,252,255,.98) 0%,rgba(118,205,255,.84) 38%,rgba(118,205,255,.08) 72%,rgba(118,205,255,0) 100%);box-shadow:0 0 18px rgba(118,205,255,.42),0 0 34px rgba(118,205,255,.22)}
body.state-C .vehicleStage .chargeSpark svg,body.state-D .vehicleStage .chargeSpark svg{fill:#ddf4ff}
body.state-E .vehicleStage .chargeAura,body.state-F .vehicleStage .chargeAura{background:radial-gradient(circle at 50% 50%,rgba(255,151,151,.22) 0%,rgba(255,151,151,.10) 34%,rgba(255,151,151,0) 72%)}
body.state-E .vehicleStage .chargeBeam,body.state-F .vehicleStage .chargeBeam{background:linear-gradient(180deg,rgba(255,140,140,0) 0%,rgba(255,140,140,.14) 14%,rgba(255,238,238,.92) 34%,rgba(255,140,140,.16) 56%,rgba(255,140,140,0) 100%)}
body.state-E .vehicleStage .chargePulse,body.state-F .vehicleStage .chargePulse{background:radial-gradient(circle,rgba(255,248,248,.98) 0%,rgba(255,150,150,.82) 38%,rgba(255,120,120,.08) 72%,rgba(255,120,120,0) 100%);box-shadow:0 0 18px rgba(255,142,142,.38),0 0 34px rgba(255,142,142,.2)}
body.state-E .vehicleStage .chargeSpark svg,body.state-F .vehicleStage .chargeSpark svg{fill:#fff1f1}
.metricCardDate{margin:2px 0 14px;padding:11px 14px;border-radius:18px;text-align:center;font-size:16px;font-weight:800;letter-spacing:-.02em;color:#d5f0ff;background:linear-gradient(180deg,rgba(12,32,50,.68),rgba(7,19,31,.48));border:1px solid rgba(151,210,255,.10);box-shadow:inset 0 1px 0 rgba(255,255,255,.04)}
.stationMeta{margin:0 0 14px;display:grid;gap:8px}
.stationMetaLine{padding:10px 12px;border-radius:16px;background:linear-gradient(180deg,rgba(12,32,50,.54),rgba(7,19,31,.38));border:1px solid rgba(151,210,255,.08);box-shadow:inset 0 1px 0 rgba(255,255,255,.03)}
.stationMetaLabel{font-size:11px;font-weight:800;letter-spacing:.08em;text-transform:uppercase;color:#8cb9cb}
.stationMetaValue{margin-top:4px;color:#effcff;font-size:15px;font-weight:800;line-height:1.35;word-break:break-word}
.stationMetaValue.muted{color:#c4e7f4;font-size:13px;font-weight:700}
.metricLabel{font-size:17px;color:#91cbb9;font-weight:700;}
.metricHero{margin-top:10px;font-size:46px;line-height:1;font-weight:800;letter-spacing:-.04em;color:#f1fffb;}
.metricHero .unit{font-size:.58em;font-weight:700;letter-spacing:-.01em;color:#7fe5c1;}
.metricsGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px 16px;}
.metricCell{min-width:0;padding:14px 15px 15px;border-radius:20px;background:linear-gradient(180deg,rgba(12,32,50,.64),rgba(8,22,36,.42));border:1px solid rgba(151,210,255,.10);box-shadow:inset 0 1px 0 rgba(255,255,255,.04),0 10px 22px rgba(2,10,18,.12);}
.metricCell--power{grid-column:1/-1;padding:15px 18px 17px;border-radius:22px;background:linear-gradient(180deg,rgba(14,38,60,.82),rgba(8,22,36,.52));border:1px solid rgba(151,210,255,.13);box-shadow:inset 0 1px 0 rgba(255,255,255,.05),0 12px 24px rgba(2,10,18,.14);text-align:center;}
.metricCell--power .metricHead{justify-content:center;font-size:15px;}
.metricCell--power .metricValue{margin-top:10px;font-size:38px;line-height:1;color:#eefcf8}
.metricCell--power .metricValue .unit{font-size:.44em;font-weight:700;color:#7fe5c1}
.metricHead{display:flex;align-items:center;gap:8px;color:#9fcfc2;font-size:15px;font-weight:800;}
.metricHead svg{width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:1.9;stroke-linecap:round;stroke-linejoin:round;flex:0 0 auto;}
.metricValue{margin-top:10px;font-size:24px;font-weight:800;letter-spacing:-.03em;color:#f0fcf8;}
.metricValue .unit{font-size:.82em;color:#6fd8b1;}
.installBtn{display:none;margin-top:14px;width:100%;padding:14px 16px;border-radius:18px;border:1px solid rgba(76,210,166,.16);background:linear-gradient(135deg,rgba(13,40,53,.96),rgba(7,22,31,.9));color:#dcfff2;font-size:14px;font-weight:800;box-shadow:var(--shadowSoft);}
@keyframes screenEnter{0%{opacity:0;transform:translateY(18px)}100%{opacity:1;transform:translateY(0)}}
@keyframes fadeLift{0%{opacity:0;transform:translateY(14px)}100%{opacity:1;transform:translateY(0)}}
@keyframes floatGlowA{0%,100%{transform:translate3d(0,0,0) scale(1)}50%{transform:translate3d(22px,18px,0) scale(1.08)}}
@keyframes floatGlowB{0%,100%{transform:translate3d(0,0,0) scale(1)}50%{transform:translate3d(-18px,-14px,0) scale(1.1)}}
@keyframes livePulse{0%,100%{transform:scale(1)}50%{transform:scale(1.22)}}
@keyframes chargeTravel{0%{top:74%;opacity:0;transform:translateX(-50%) scale(.76)}12%{opacity:calc(.22 + var(--chargeOpacity,.72))}50%{opacity:calc(.35 + var(--chargeOpacity,.72)*.75)}100%{top:12%;opacity:0;transform:translateX(-50%) scale(1.06)}}
@keyframes chargeBeamFlow{0%{transform:translateY(10%) scaleY(.92);opacity:.05}45%{transform:translateY(-2%) scaleY(1);opacity:calc(.12 + var(--chargeOpacity,.72)*.72)}100%{transform:translateY(-12%) scaleY(1.05);opacity:.04}}
@keyframes sparkFlash{0%,100%{transform:translate(-50%,-50%) rotate(-6deg) scale(.88);opacity:.08}38%{transform:translate(-50%,-50%) rotate(2deg) scale(1.08);opacity:calc(.16 + var(--chargeOpacity,.72)*.7)}52%{transform:translate(-50%,-50%) rotate(-4deg) scale(.94);opacity:.2}}
@media(max-width:780px){.screenShell{padding:12px 10px 20px}.app{max-width:100%;min-height:calc(100vh - 30px)}.metricCard{max-width:94%;margin-top:18px;padding:18px 18px}.vehicleStage{width:min(74%,210px)}}
@media(max-width:390px){.screenShell{padding:10px 8px 18px}.statusPill{padding:10px 13px;font-size:14px}.dateLine{font-size:18px}.locationLine{font-size:16px}.metricHero{font-size:40px}.metricValue{font-size:19px}.metricCard{margin-top:16px;padding:16px 16px}.metricCardArt{margin-bottom:12px}.vehicleStage{width:min(72%,190px)}}
__USER_CUSTOM_CSS__
</style>
<link rel="manifest" href="/manifest.json">
<meta name="theme-color" content="#0a1d2f">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="RotosisEVSE">
</head><body class="state-A">
<div class="screenShell">
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

  <section class="metricCard metricCard--stats">
    <div class="metricCardArt" aria-hidden="true">
      <div class="vehicleStage" id="vehicleStage">
        <img src="/vehicle-top-art.svg" alt="">
        <div class="chargeOverlay">
          <div class="chargeAura"></div>
          <div class="chargeBeam"></div>
          <div class="chargePulse p1"></div>
          <div class="chargePulse p2"></div>
          <div class="chargePulse p3"></div>
          <div class="chargeSpark">
            <svg viewBox="0 0 64 64" aria-hidden="true"><path d="M35.6 4 15 35.5h12.8L22.8 60 49 25.8H36.4z"/></svg>
          </div>
        </div>
      </div>
    </div>
    <div class="metricCardDate" id="dateLabel">-</div>
    <div class="stationMeta">
      <div class="stationMetaLine">
        <div class="stationMetaLabel">Istasyon Adi</div>
        <div class="stationMetaValue" id="stationNameLabel">Istasyon</div>
      </div>
      <div class="stationMetaLine">
        <div class="stationMetaLabel">Uretilen Adres</div>
        <div class="stationMetaValue muted" id="stationAddrLabel">-</div>
      </div>
    </div>
    <div class="metricsGrid">
      <div class="metricCell metricCell--power">
        <div class="metricHead">
          <svg viewBox="0 0 24 24"><path d="M13 2L6 13h4l-1 9 7-11h-4z"></path></svg>
          <span>Aktar&#305;lan G&#252;&#231;</span>
        </div>
        <div class="metricValue"><span id="pwr">0.00</span> <span class="unit">kW</span></div>
      </div>
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
  </section>

  <button class="installBtn" id="installBtn">Uygulamaya Ekle</button>
</div>
</div>
<script>
let deferredPrompt=null;
const POLL_MS=2000;
const MAX_POINTS=40;
let livePoints=[];
let chartCeil=32;
let currentChargeModeId=0;
const vehicleStage=document.getElementById("vehicleStage");
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
  chargeBar:document.getElementById("chargeBar"),
  chargeBarInner:document.getElementById("chargeBarInner")
};
function escapeHtml(value){
  return String(value||"").replace(/[&<>"']/g,(ch)=>({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[ch]));
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
function setChargingVisual(state,powerKw,currentA){
  if(!vehicleStage) return;
  const waiting=state==="A";
  const charging=(state==="C"||state==="D");
  const active=waiting || charging;
  vehicleStage.classList.toggle("is-charging",active);
  const intensity=waiting
    ? .34
    : charging
      ? clamp(Math.max((Number(powerKw)||0)/11,(Number(currentA)||0)/32,.28),.28,1)
      : .16;
  const speed=(waiting?2.85:(2.9-(intensity*1.35))).toFixed(2)+"s";
  vehicleStage.style.setProperty("--chargeOpacity",intensity.toFixed(2));
  vehicleStage.style.setProperty("--chargeSpeed",speed);
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
    const limitA=Math.max(6,Number(d.limitTargetA ?? d.limitA)||32);
    const totalLimit=phaseCount*limitA;
    const loadPct=clamp((it/totalLimit)*100,0,100);
    const powerKw=((Number(d.pW)||0)/1000.0);
    const liveSession=!!d.sLive;
    const energy=(liveSession&&d.sLiveKWh!==undefined)?(Number(d.sLiveKWh)||0):(Number(d.eKWh)||0);
    const timeSec=(liveSession&&d.sLiveSec!==undefined)?(Number(d.sLiveSec)||0):(Number(d.tSec)||0);
    const activePhases=Math.max(1,[ia,ib,ic].filter(v=>v>0.5).length||phaseCount);
    const displayCurrent=activePhases>1 ? (it/activePhases) : it;
    chartCeil=Math.max(6,totalLimit);
    setText('it',displayCurrent.toFixed(1));
    setText('currentMetric',displayCurrent.toFixed(1));
    setText('i1',ia.toFixed(1)+" A");
    setText('i2',ib.toFixed(1)+" A");
    setText('i3',ic.toFixed(1)+" A");
    setText('pwr',powerKw.toFixed(2));
    setText('ekwh',energy.toFixed(1));
    setText('tsec',fmtTime(timeSec));
    setText('stationNameLabel',d.stationName||'Istasyon');
    setText('stationAddrLabel',d.stationAddr||'-');
    setText('energyMeta',liveSession?"Aktif seans":"Son okuma");
    setText('timeMeta',phaseCount+" faz");
    setText('limitA',limitA.toFixed(1));
    setText('limitMetric',limitA.toFixed(1));
    setText('limitMeta',phaseCount+" faz / "+limitA.toFixed(1)+" A limit");
    setText('phaseSummary',phaseCount+" faz • "+loadPct.toFixed(0)+"% doluluk");
    setText('gaugeLabel',phaseCount+" faz / "+limitA.toFixed(1)+" A");
    updateChargeAction(Number(d.modeId)||0);
    if(d.rLbl!==undefined) setText('relay',"R:"+d.rLbl);
    if(d.state!==undefined) updateState(d.state);
    setChargingVisual(String(d.state||"A"),powerKw,displayCurrent);
    if(d.ip!==undefined) setText('ip',d.ip);
    if(d.host!==undefined) setText('host',d.host);
    setGauge(loadPct);
    setText('ts',"Son güncelleme: "+new Date().toLocaleTimeString("tr-TR",{hour:"2-digit",minute:"2-digit",second:"2-digit"}));
    renderAlarm(d.alarmLv||0, d.alarmTxt||"Sistem normal", d.state||"A", !!d.staOk);
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
  "background_color": "#07131f",
  "theme_color": "#0a1d2f",
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
const CACHE_NAME = "evse-pwa-v7";
const ASSETS = ["/", "/manifest.json", "/app-icon.svg", "/vehicle-top-art.svg"];

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
<title>RotosisEVSE Settings</title>
<style>
body{font-family:Arial;margin:0;background:#09111d;color:#e8eefc}
.wrap{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(320px,.85fr);gap:12px;padding:12px}
@media(max-width:960px){.wrap{grid-template-columns:1fr}}
.card{background:#111a2b;border:1px solid #20304a;border-radius:14px;padding:12px}
.navCard{grid-column:1/-1}
h2{margin:0 0 10px 0;font-size:14px;color:#b7c5e6}
.kv{display:grid;grid-template-columns:150px 1fr;gap:8px;margin:6px 0}
@media(max-width:560px){.kv{grid-template-columns:1fr}}
.k{color:#b7c5e6;font-size:12px}
.v{font-weight:700}
.mono{font-family:monospace;color:#00ffaa}
input,select,textarea{width:100%;padding:8px;border-radius:10px;border:1px solid #20304a;background:#0c1424;color:#e8eefc;box-sizing:border-box}
textarea{min-height:180px;resize:vertical;font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,"Courier New",monospace;line-height:1.45}
.btns{display:flex;flex-wrap:wrap;gap:8px}
button{padding:8px 10px;border-radius:10px;border:1px solid #20304a;background:#0c1424;color:#e8eefc;cursor:pointer}
.primary{background:rgba(0,200,150,.18);border-color:rgba(0,200,150,.45)}
.danger{background:rgba(255,77,77,.14);border-color:rgba(255,77,77,.40)}
.small{font-size:11px;color:#b7c5e6;line-height:1.5}
.sep{height:1px;background:#20304a;margin:12px 0}
.hero{background:linear-gradient(180deg,rgba(0,200,150,.10),rgba(0,0,0,0));border:1px solid #20304a;border-radius:12px;padding:10px;margin-bottom:10px}
.heroTop{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:6px}
.badge{font-family:monospace;font-size:12px;padding:4px 8px;border-radius:999px;border:1px solid rgba(0,200,150,.45);background:rgba(0,200,150,.12);color:#00ffaa}
.grid3{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.grid2{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
@media(max-width:680px){.grid3,.grid2{grid-template-columns:1fr}}
.hintBox{padding:10px 12px;border-radius:12px;background:rgba(0,200,150,.08);border:1px solid rgba(0,200,150,.18);color:#bde9dd;font-size:12px;line-height:1.55}
.pageSection{display:block}
.navRow{display:flex;flex-wrap:wrap;gap:8px}
.navBtn{display:inline-flex;align-items:center;justify-content:center;padding:8px 10px;border-radius:10px;border:1px solid #20304a;background:#0c1424;color:#e8eefc;text-decoration:none}
.navBtn.active{background:rgba(0,200,150,.18);border-color:rgba(0,200,150,.45)}
</style></head><body>

<div class="wrap">
  <div class="card navCard">
    <h2>YONLENDIRME</h2>
    <div class="navRow">
      <a id="navAdmin" class="navBtn" href="/admin">CANLI PANEL</a>
      <a id="navCalibration" class="navBtn" href="/calibration">KALIBRASYON</a>
      <a id="navWifi" class="navBtn" href="/wifi">WIFI AYARLARI</a>
      <a class="navBtn" href="/">KULLANICI EKRANI</a>
    </div>
    <div class="small" style="margin-top:10px">Admin panelde sadece canli veriler tutulur. Ayarlar ayri pencerelerden yonetilir.</div>
  </div>

  <div class="card pageSection" id="liveCard">
    <h2>CANLI VERILER</h2>
    <div class="kv"><div class="k">State (Stable/Raw)</div><div class="v mono"><span id="sStb">-</span> / <span id="sRaw">-</span></div></div>
    <div class="kv"><div class="k">CP High / Low</div><div class="v mono"><span id="cH">-</span> / <span id="cL">-</span></div></div>
    <div class="kv"><div class="k">ADC High / Low</div><div class="v mono"><span id="aH">-</span> / <span id="aL">-</span></div></div>
    <div class="kv"><div class="k">PP GPIO16</div><div class="v mono"><span id="ppV">-</span> / <span id="ppRaw">-</span></div></div>
    <div class="sep"></div>
    <h2>KULLANICI EKRANI OZEL CSS</h2>
    <div class="small">Buraya yazdigin CSS, kullanici ekranindaki tum elemanlari ezebilir. Ornek: `body{background:#000}` veya `.metricCard{border-radius:36px}`</div>
    <div class="kv"><div class="k">Ozel CSS</div><div class="v"><textarea id="userCssSet" onfocus="p()" onblur="r()" placeholder=".metricCard{outline:1px solid red}">__USER_CSS_TEXT__</textarea></div></div>
    <div class="btns" style="margin-top:12px">
      <button class="primary" onclick="applyUserCss()">CSS KAYDET</button>
      <button onclick="window.open('/','_blank')">KULLANICI EKRANINI AC</button>
    </div>
  </div>

  <div class="card pageSection" id="calibrationCard">
    <div class="sep"></div>
    <h2>ZAMANLAMA</h2>
    <div class="kv"><div class="k">Loop dongusu (ms)</div><div class="v"><input id="lInt" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Role acma (ms)</div><div class="v"><input id="onD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Role birakma (ms)</div><div class="v"><input id="offD" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Stable count</div><div class="v"><input id="stb" onfocus="p()" onblur="r()"></div></div>
    <div class="kv"><div class="k">Akim limiti (A)</div><div class="v"><input id="limitASet" min="6" max="32" step="1" onfocus="p()" onblur="r()"></div></div>

    <div class="sep"></div>
    <h2>CP KALIBRASYON</h2>
    <div class="small">State gecis esikleri (TH_B/C/D/E, stable) otomatik degismez. Bunlari sen ayarlarsin; UYGULA ile kalici kaydedilir. Divider da sadece senin girdigin degerdir.</div>
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

    <div class="sep"></div>
    <h2>ISTASYON KONUMU</h2>
    <div class="kv"><div class="k">Istasyon adi</div><div class="v"><input id="stationNameSet" onfocus="p()" onblur="r()" placeholder="Istasyon ABC123"></div></div>
    <div class="kv"><div class="k">Latitude / Longitude</div><div class="v grid2"><input id="mapLat" onfocus="p()" onblur="r()" placeholder="37.94559"><input id="mapLng" onfocus="p()" onblur="r()" placeholder="32.58082"></div></div>
    <div class="kv"><div class="k">Uretilen adres</div><div class="v mono"><span id="stationAddr">-</span></div></div>
    <div class="small">Istasyon adi bos birakilirsa cihaz MAC bilgisinden otomatik uretilen varsayilan isim kullanilir.</div>
    <div class="small">Konum kaydedildiginde harita merkezi ve istasyon adres metni bu koordinatlara gore guncellenir.</div>

    <div class="btns" style="margin-top:12px">
      <button class="primary" onclick="applyCal()">UYGULA</button>
      <button onclick="pull(true)">YENILE</button>
    </div>
  </div>

  <div class="card pageSection" id="statusCard">
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

  <div class="card pageSection" id="wifiCard">
    <h2>WIFI BAGLANTISI</h2>
    <div class="kv"><div class="k">Ozel profil</div><div class="v"><label><input type="checkbox" id="wifiEnabled" onfocus="p()" onblur="r()"> Etkin</label></div></div>
    <div class="kv"><div class="k">SSID / Sifre</div><div class="v grid2"><input id="wifiSsidCfg" onfocus="p()" onblur="r()" placeholder="SSID"><input id="wifiPassCfg" type="password" onfocus="p()" onblur="r()" placeholder="Sifre"></div></div>
    <div class="kv"><div class="k">IP modu</div><div class="v"><select id="wifiModeCfg" onfocus="p()" onblur="r()" onchange="toggleWifiModeFields()"><option value="dhcp">DHCP</option><option value="static">Statik</option></select></div></div>
    <div id="wifiStaticFields">
      <div class="kv"><div class="k">IP / Gateway</div><div class="v grid2"><input id="wifiIpCfg" onfocus="p()" onblur="r()" placeholder="192.168.1.200"><input id="wifiGwCfg" onfocus="p()" onblur="r()" placeholder="192.168.1.1"></div></div>
      <div class="kv"><div class="k">Subnet / DNS1</div><div class="v grid2"><input id="wifiSubnetCfg" onfocus="p()" onblur="r()" placeholder="255.255.255.0"><input id="wifiDns1Cfg" onfocus="p()" onblur="r()" placeholder="8.8.8.8"></div></div>
      <div class="kv"><div class="k">DNS2</div><div class="v"><input id="wifiDns2Cfg" onfocus="p()" onblur="r()" placeholder="1.1.1.1"></div></div>
    </div>
    <div class="btns">
      <button onclick="scanWifiList()">WIFI TARA</button>
      <button class="primary" onclick="applyWifiSettings()">WIFI AYARLARINI KAYDET</button>
    </div>
    <div class="small" id="wifiScanStatus">Tarama hazir degil</div>
    <div class="small" id="wifiCurrentMode">Baglanti profili: varsayilan liste</div>
    <div class="small" id="wifiConnectedInfo">Mevcut baglanti: bagli degil</div>
    <div id="wifiScanList" class="small"></div>
  </div>

  <div class="card pageSection" id="testCard">
    <div class="sep"></div>

    <h2>ROLE</h2>
    <div class="btns">
      <button class="primary" onclick="send('/relay?on=1')">MANUEL AC</button>
      <button class="danger" onclick="send('/relay?on=0')">MANUEL BIRAK</button>
      <button onclick="send('/relay_auto?en=1')">AUTO AC</button>
    </div>
    <div class="small">Not: Manuel komut auto akisina aninda mudahale eder.</div>
  </div>

  <div class="card pageSection" id="otaCard">
    <div class="sep"></div>
    <h2>SIFIRLAMA GECMISI</h2>
    <div class="kv"><div class="k">Toplam</div><div class="v mono"><span id="rstTotal">0</span></div></div>
    <div class="kv"><div class="k">Anlik / Gecmis</div><div class="v mono"><span id="rstNow">0</span> / <span id="rstHist">0</span></div></div>
    <div class="kv"><div class="k">Son islem</div><div class="v mono"><span id="rstLastMode">YOK</span> @ <span id="rstLastSec">0</span>s</div></div>

    <div class="sep"></div>
    <h2>OTA TESHIS</h2>
    <div class="kv"><div class="k">FW / Remote</div><div class="v mono"><span id="otaCurVer">-</span> / <span id="otaRemoteVer">-</span></div></div>
    <div class="kv"><div class="k">Calisan bolum</div><div class="v mono"><span id="otaPart">-</span></div></div>
    <div class="kv"><div class="k">Image state</div><div class="v mono"><span id="otaImgState">-</span></div></div>
    <div class="kv"><div class="k">Durum</div><div class="v mono"><span id="otaStatus">-</span></div></div>
    <div class="kv"><div class="k">Son kontrol</div><div class="v mono"><span id="otaAge">-</span></div></div>
    <div class="kv"><div class="k">Hata</div><div class="v mono"><span id="otaError">Hata yok</span></div></div>
    <div class="btns" style="margin-top:10px">
      <button class="primary" onclick="window.location='/update'">BIN YUKLE</button>
      <button class="primary" onclick="runOtaCheckAdmin()">OTA KONTROL ET</button>
      <button class="primary" id="otaInstallBtn" onclick="runOtaInstall()" disabled>GUNCELLEMEYI YUKLE</button>
      <button onclick="runBootPrev()">ONCEKI OTA'YA DON</button>
      <button class="danger" onclick="runBootFactory()">FACTORY'E DON</button>
    </div>
    <div class="small">Not: OTA kontrolu yeni surumu sadece tespit eder. Yukleme icin ayrica onay vermen gerekir. `Onceki OTA'ya don` aktif olmayan OTA slotunu dener. Onceki surum oradaysa geri donersin. `Factory'e don` ise USB ile yukledigin kurtarma surumunu acar.</div>
  </div>
</div>

<script>
const PAGE_MODE="__PAGE_MODE__";
let paused = false;
let t;

function p(){ paused = true; clearTimeout(t); }
function r(){ t = setTimeout(() => { paused = false; }, 3000); }
function send(path){ fetch(path).then(() => pull(false)); }
function num(v, fallback = 0){
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}
function setInput(id, value, force){
  const el = document.getElementById(id);
  if(!el) return;
  if(el.type === 'checkbox'){
    if(document.activeElement !== el || force) el.checked = !!value;
    return;
  }
  if(el && (document.activeElement !== el || force)) el.value = value;
}
function fmtTime(sec){
  const m = Math.floor(sec / 60);
  const s = sec % 60;
  return String(m).padStart(2, "0") + ":" + String(s).padStart(2, "0");
}
function fmtOtaAge(ms){
  const sec = Math.max(0, Math.round((Number(ms) || 0) / 1000));
  if(sec === 0) return "hemen simdi";
  if(sec < 60) return sec + " sn once";
  const min = Math.floor(sec / 60);
  const rem = sec % 60;
  return min + " dk " + rem + " sn once";
}
function compareVersions(a,b){
  const pa=String(a||"").split(".").map(x=>parseInt(x,10)||0);
  const pb=String(b||"").split(".").map(x=>parseInt(x,10)||0);
  const len=Math.max(pa.length,pb.length,4);
  for(let i=0;i<len;i++){
    const av=pa[i]||0;
    const bv=pb[i]||0;
    if(av>bv) return 1;
    if(av<bv) return -1;
  }
  return 0;
}
function updateOtaInstallButton(d){
  const btn=document.getElementById('otaInstallBtn');
  if(!btn) return;
  const hasUpdate=compareVersions(d.otaRemote,d.otaCur)>0;
  btn.disabled=!hasUpdate;
  btn.textContent=hasUpdate?'GUNCELLEMEYI YUKLE':'GUNCEL';
}
function toggleWifiModeFields(){
  const wrap=document.getElementById('wifiStaticFields');
  const isStatic=(document.getElementById('wifiModeCfg').value==='static');
  if(wrap) wrap.style.display=isStatic?'block':'none';
}
function applyWifiStatus(d){
  setInput('wifiEnabled', !!d.wifiCfgEnabled, true);
  setInput('wifiSsidCfg', d.wifiCfgSsid || '', true);
  setInput('wifiPassCfg', '', true);
  setInput('wifiModeCfg', d.wifiCfgMode || 'dhcp', true);
  setInput('wifiIpCfg', d.wifiCfgIp || '', true);
  setInput('wifiGwCfg', d.wifiCfgGw || '', true);
  setInput('wifiSubnetCfg', d.wifiCfgSubnet || '', true);
  setInput('wifiDns1Cfg', d.wifiCfgDns1 || '', true);
  setInput('wifiDns2Cfg', d.wifiCfgDns2 || '', true);
  toggleWifiModeFields();
  const modeLine=document.getElementById('wifiCurrentMode');
  if(modeLine){
    modeLine.textContent = d.wifiCfgEnabled
      ? ('Baglanti profili: ' + (d.wifiCfgSsid || '-') + ' / ' + ((d.wifiCfgMode || 'dhcp').toUpperCase()))
      : 'Baglanti profili: varsayilan liste';
  }
  const connectedLine=document.getElementById('wifiConnectedInfo');
  if(connectedLine){
    connectedLine.textContent = (d.wifiSsid && d.wifiSsid !== '-')
      ? ('Mevcut baglanti: ' + d.wifiSsid + ((d.wifiLoc && d.wifiLoc !== '-') ? (' / ' + d.wifiLoc) : ''))
      : 'Mevcut baglanti: bagli degil';
  }
}
function showSection(id, visible){
  const el=document.getElementById(id);
  if(el) el.style.display=visible?'block':'none';
}
function setActiveNav(id){
  ['navAdmin','navCalibration','navWifi'].forEach((navId)=>{
    const el=document.getElementById(navId);
    if(el) el.classList.toggle('active', navId===id);
  });
}
function applyPageMode(){
  const mode=PAGE_MODE || 'admin';
  showSection('liveCard', mode==='admin');
  showSection('statusCard', mode==='admin');
  showSection('calibrationCard', mode==='calibration');
  showSection('wifiCard', mode==='wifi');
  showSection('testCard', mode==='calibration');
  showSection('otaCard', mode==='wifi');
  if(mode==='wifi'){
    setActiveNav('navWifi');
  }else if(mode==='calibration'){
    setActiveNav('navCalibration');
  }else{
    setActiveNav('navAdmin');
  }
}
function renderWifiScanList(items){
  const host=document.getElementById('wifiScanList');
  if(!host) return;
  if(!items || !items.length){
    host.innerHTML='Tarama sonucu bos';
    return;
  }
  host.innerHTML=items.map((item)=>{
    const ssid=String(item.ssid||'').replace(/[&<>"]/g,(ch)=>({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;"}[ch]));
    return '<div style="display:flex;justify-content:space-between;gap:8px;margin:6px 0;padding:8px;border:1px solid #20304a;border-radius:10px;">'+
      '<span>'+ssid+'</span><span>'+String(item.rssi||0)+' dBm</span><button type="button" onclick="pickWifiSsid('+JSON.stringify(String(item.ssid||''))+')">Sec</button></div>';
  }).join('');
}
function pickWifiSsid(ssid){
  const el=document.getElementById('wifiSsidCfg');
  if(el) el.value=ssid;
}
function scanWifiList(){
  const status=document.getElementById('wifiScanStatus');
  if(status) status.textContent='Tarama yapiliyor...';
  fetch('/wifi_scan', {cache:'no-store'})
    .then(r=>r.json())
    .then(d=>{
      renderWifiScanList(d.items||[]);
      if(status) status.textContent='Tarama tamamlandi';
    })
    .catch(()=>{
      if(status) status.textContent='Tarama basarisiz';
    });
}
function applyWifiSettings(){
  const q=new URLSearchParams();
  q.append('wifiEnabled', document.getElementById('wifiEnabled').checked ? '1' : '0');
  q.append('wifiSsid', document.getElementById('wifiSsidCfg').value);
  q.append('wifiPass', document.getElementById('wifiPassCfg').value);
  q.append('wifiDhcp', document.getElementById('wifiModeCfg').value === 'static' ? '0' : '1');
  q.append('wifiIp', document.getElementById('wifiIpCfg').value);
  q.append('wifiGw', document.getElementById('wifiGwCfg').value);
  q.append('wifiSubnet', document.getElementById('wifiSubnetCfg').value);
  q.append('wifiDns1', document.getElementById('wifiDns1Cfg').value);
  q.append('wifiDns2', document.getElementById('wifiDns2Cfg').value);
  fetch('/wifi_apply?' + q.toString(), {cache:'no-store'})
    .then(r=>r.json())
    .then(()=>{
      alert('Wi-Fi ayarlari kaydedildi. Cihaz yeniden baglanmaya calisiyor.');
      paused=false;
      pull(true);
    })
    .catch(()=>alert('Wi-Fi ayarlari kaydedilemedi'));
}
function applyUserCss(){
  const q=new URLSearchParams();
  q.append('userCss', document.getElementById('userCssSet').value);
  if(document.activeElement) document.activeElement.blur();
  fetch('/calib_apply?' + q.toString())
    .then(() => {
      alert('Kullanici ekrani CSS kaydedildi');
      paused=false;
      pull(true);
    })
    .catch(() => alert('Kullanici ekrani CSS kaydedilemedi'));
}
function runOtaCheckAdmin(){
  fetch('/ota_check', {cache:'no-store'}).then(() => {
    document.getElementById('otaStatus').textContent = 'check_requested';
  });
}
function runOtaInstall(){
  const cur=document.getElementById('otaCurVer').textContent||'-';
  const remote=document.getElementById('otaRemoteVer').textContent||'-';
  if(compareVersions(remote,cur)<=0){
    alert('Yuklenecek yeni OTA surumu bulunmuyor.');
    return;
  }
  if(!confirm('Yeni OTA surumu yüklensin ve cihaz yeniden başlatılsın mı?')) return;
  fetch('/ota_install', {cache:'no-store'})
    .then(r => r.json())
    .then(() => {
      document.getElementById('otaStatus').textContent='install_requested';
      document.getElementById('otaError').textContent='Admin onayi verildi, OTA yukleme baslatiliyor';
    })
    .catch(() => alert('OTA yukleme istegi gonderilemedi'));
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
  fetch('/status', {cache:'no-store'}).then(r => r.json()).then(d => {
    document.getElementById('sStb').textContent = d.state;
    document.getElementById('sRaw').textContent = d.stateRaw;
    document.getElementById('cH').textContent = num(d.cpHigh).toFixed(2) + 'V';
    document.getElementById('cL').textContent = num(d.cpLow).toFixed(2) + 'V';
    document.getElementById('aH').textContent = num(d.adcHigh).toFixed(3) + 'V';
    document.getElementById('aL').textContent = num(d.adcLow).toFixed(3) + 'V';
    document.getElementById('ppV').textContent = num(d.ppVolt).toFixed(3) + 'V';
    document.getElementById('ppRaw').textContent = String(num(d.ppRaw, 0));
    document.getElementById('i1').textContent = num(d.ia).toFixed(2);
    document.getElementById('i2').textContent = num(d.ib).toFixed(2);
    document.getElementById('i3').textContent = num(d.ic).toFixed(2);
    document.getElementById('pwr').textContent = (num(d.pW) / 1000.0).toFixed(2);
    document.getElementById('ekwh').textContent = num(d.eKWh).toFixed(3);
    document.getElementById('tsec').textContent = fmtTime(num(d.tSec, 0));
    document.getElementById('phase').textContent = num(d.phase, 1) + 'F';
    document.getElementById('wifiSsid').textContent = d.wifiSsid || '-';
    document.getElementById('wifiLoc').textContent = d.wifiLoc || '-';
    document.getElementById('ip').textContent = d.ip || '-';
    document.getElementById('host').textContent = d.host || '-';
    document.getElementById('rLbl').textContent = d.rLbl || '-';
    document.getElementById('stationAddr').textContent = d.stationAddr || '-';
    document.getElementById('rstTotal').textContent = d.rstTotal || 0;
    document.getElementById('rstNow').textContent = d.rstNow || 0;
    document.getElementById('rstHist').textContent = d.rstHist || 0;
    document.getElementById('rstLastMode').textContent = d.rstLastMode || 'YOK';
    document.getElementById('rstLastSec').textContent = d.rstLastSec || 0;
    document.getElementById('otaCurVer').textContent = d.otaCur || '-';
    document.getElementById('otaRemoteVer').textContent = d.otaRemote || '-';
    document.getElementById('otaPart').textContent = d.otaPart || '-';
    document.getElementById('otaImgState').textContent = d.otaImgState || '-';
    document.getElementById('otaStatus').textContent = d.otaStatus || '-';
    document.getElementById('otaAge').textContent = fmtOtaAge(d.otaAgeMs);
    document.getElementById('otaError').textContent = (d.otaErr && d.otaErr.length) ? d.otaErr : 'Hata yok';
    updateOtaInstallButton(d);
    applyWifiStatus(d);
    document.getElementById('badgeState').textContent = 'STATE:' + (d.state || '-');
    const relayBadge = document.getElementById('badgeRelay');
    relayBadge.textContent = 'R: ' + (d.rLbl || '-');
    relayBadge.style.backgroundColor = (d.state === 'C') ? 'rgba(0,255,170,0.3)' : '';

    document.getElementById('liveIa').textContent = num(d.ia).toFixed(2);
    document.getElementById('liveIb').textContent = num(d.ib).toFixed(2);
    document.getElementById('liveIc').textContent = num(d.ic).toFixed(2);
    document.getElementById('liveIAvg').textContent = num(d.iAvg).toFixed(2);

    setInput('lInt', d.lInt, force);
    setInput('onD', d.onD, force);
    setInput('offD', d.offD, force);
    setInput('stb', d.stable, force);
    setInput('limitASet', (d.limitTargetA !== undefined) ? d.limitTargetA : d.limitA, force);
    setInput('div', d.div, force);
    setInput('thb', d.thb, force);
    setInput('thc', d.thc, force);
    setInput('thd', d.thd, force);
    setInput('the', d.the, force);
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
    setInput('stationNameSet', d.stationName || '', force);
    setInput('mapLat', d.mapLat, force);
    setInput('mapLng', d.mapLng, force);
  });
}
function applyCal(){
  const q = new URLSearchParams();
  q.append('lInt', document.getElementById('lInt').value);
  q.append('onD', document.getElementById('onD').value);
  q.append('offD', document.getElementById('offD').value);
  q.append('s', document.getElementById('stb').value);
  q.append('limitA', document.getElementById('limitASet').value);
  q.append('div', document.getElementById('div').value);
  q.append('thb', document.getElementById('thb').value);
  q.append('thc', document.getElementById('thc').value);
  q.append('thd', document.getElementById('thd').value);
  q.append('the', document.getElementById('the').value);
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
  q.append('stationName', document.getElementById('stationNameSet').value);
  q.append('mapLat', document.getElementById('mapLat').value);
  q.append('mapLng', document.getElementById('mapLng').value);
  q.append('userCss', document.getElementById('userCssSet').value);
  if(document.activeElement) document.activeElement.blur();
  fetch('/calib_apply?' + q.toString()).then(() => {
    alert('Tamam');
    paused = false;
    pull(true);
  });
}
applyPageMode();
setInterval(() => pull(false), 3000);
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
  WiFi.mode(WIFI_STA);
  refreshDeviceIdentity();
  WiFi.setHostname(currentHostName());
  WiFi.softAPdisconnect(true);
  Serial.println("[WiFi] AP kapali, sadece STA modu aktif.");
  Serial.print("[WiFi] Hostname: ");
  Serial.println(currentHostName());

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
  rebuildKnownWifiList();
  for (size_t i = 0; i < (sizeof(kKnownWifis) / sizeof(kKnownWifis[0])); i++) {
    if (!hasText(kKnownWifis[i].ssid)) continue;
    addedWifiCount++;
    Serial.printf(" - %s (%s)\n", kKnownWifis[i].location, kKnownWifis[i].ssid);
  }
  if (addedWifiCount == 0) {
    Serial.println(" - STA listesi bos.");
  }

  bool connected = connectConfiguredWifi(5000);
  if (!connected && addedWifiCount > 0 && wifiMulti.run(1500) == WL_CONNECTED) {
    connected = true;
  }

  if (connected) {
    Serial.println("");
    Serial.println("WiFi Baglandi!");
    Serial.print("Konum: ");
    Serial.println(wifiLocationForSsid(WiFi.SSID()));
    Serial.println("IP adresi: ");
    Serial.println(WiFi.localIP());
  }

  Serial.println("[WiFi] Kendi AP yayini kapali.");
  if (s_mdnsEnabled) {
    refreshMdns();
  } else {
    Serial.println("[mDNS] Test icin gecici olarak devre disi");
  }
}

// 3) HTTP handler'lari.
// Her endpoint kendi verisini veya komutunu burada uretir.
static void handleRoot() {
  noteWebActivity();
  String html(FPSTR(USER_HTML));
  html.replace("__USER_CUSTOM_CSS__", styleEscape(s_userScreenCustomCss));
  noteHttpResponseSent();
  server.send(200, "text/html", html);
}
static void sendMainPage(const char* pageMode) {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  String html(FPSTR(MAIN_HTML));
  html.replace("__PAGE_MODE__", pageMode ? pageMode : "admin");
  html.replace("__USER_CSS_TEXT__", htmlEscape(s_userScreenCustomCss));
  noteHttpResponseSent();
  server.send(200, "text/html", html);
}
static void handleAdmin() { sendMainPage("admin"); }
static void handleCalibrationPage() { sendMainPage("calibration"); }
static void handleWifiPage() { sendMainPage("wifi"); }
static void handlePing() { noteWebActivity(); noteHttpResponseSent(); server.send(200, "text/plain", "OK"); }
static void handleManifest() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
static void handleServiceWorker() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/javascript", SERVICE_WORKER_JS); }
static void handleAppIcon() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "image/svg+xml", APP_ICON_SVG); }
static void handleVehicleTopArt() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "image/svg+xml", VEHICLE_TOP_ART_SVG); }
static void handleOtaCheck() {
  noteWebActivity();
  OTA_Manager::triggerCheckNow();
  noteHttpResponseSent();
  server.send(200, "application/json", "{\"ok\":1}");
}

static void handleOtaInstall() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  OTA_Manager::triggerInstallNow();
  noteHttpResponseSent();
  server.send(200, "application/json", "{\"ok\":1}");
}

static void handleWifiScan() {
  noteWebActivity();
  if (!requireAdminAuth()) return;

  int count = WiFi.scanNetworks(false, true);
  if (count < 0) count = 0;

  String json = "{\"items\":[";
  for (int i = 0; i < count; ++i) {
    if (i > 0) json += ",";
    json += "{\"ssid\":\"";
    json += jsonEscape(WiFi.SSID(i));
    json += "\",\"rssi\":";
    json += String(WiFi.RSSI(i));
    json += ",\"enc\":";
    json += String((int)WiFi.encryptionType(i));
    json += "}";
  }
  json += "]}";
  s_lastWifiScanJson = json;
  s_lastWifiScanMs = millis();
  noteHttpResponseSent();
  server.send(200, "application/json", s_lastWifiScanJson);
}

static void handleWifiApply() {
  noteWebActivity();
  if (!requireAdminAuth()) return;

  bool changed = false;
  if (server.hasArg("wifiEnabled")) {
    s_customWifiEnabled = (server.arg("wifiEnabled") == "1");
    changed = true;
  }
  if (server.hasArg("wifiSsid")) {
    s_customWifiSsid = server.arg("wifiSsid");
    s_customWifiSsid.trim();
    changed = true;
  }
  if (server.hasArg("wifiPass")) {
    s_customWifiPassword = server.arg("wifiPass");
    changed = true;
  }
  if (server.hasArg("wifiDhcp")) {
    s_wifiUseDhcp = (server.arg("wifiDhcp") != "0");
    changed = true;
  }

  IPAddress parsed;
  if (server.hasArg("wifiIp") && parseIpAddressArg(server.arg("wifiIp"), parsed)) {
    s_staticIp = parsed;
    changed = true;
  }
  if (server.hasArg("wifiGw") && parseIpAddressArg(server.arg("wifiGw"), parsed)) {
    s_staticGateway = parsed;
    changed = true;
  }
  if (server.hasArg("wifiSubnet") && parseIpAddressArg(server.arg("wifiSubnet"), parsed)) {
    s_staticSubnet = parsed;
    changed = true;
  }
  if (server.hasArg("wifiDns1") && parseIpAddressArg(server.arg("wifiDns1"), parsed)) {
    s_staticDns1 = parsed;
    changed = true;
  }
  if (server.hasArg("wifiDns2") && parseIpAddressArg(server.arg("wifiDns2"), parsed)) {
    s_staticDns2 = parsed;
    changed = true;
  }

  if (changed) {
    saveWifiSettings();
    reconnectWifiNow();
  }

  noteHttpResponseSent();
  server.send(200, "application/json", "{\"ok\":1}");
}

static void scheduleDeferredRestart() {
  s_manualOtaRebootPending = true;
  s_manualOtaRebootAtMs = millis() + 300;
}

static void handleBootFactory() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  if (!OTA_Manager::selectFactoryBootPartition()) {
    noteHttpResponseSent();
    server.send(500, "text/plain", OTA_Manager::lastErrorText());
    return;
  }
  scheduleDeferredRestart();
  noteHttpResponseSent();
  server.send(200, "text/plain", "Factory secildi, cihaz yeniden baslatiliyor");
}

static void handleBootPrev() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  if (!OTA_Manager::selectAlternateOtaBootPartition()) {
    noteHttpResponseSent();
    server.send(409, "text/plain", OTA_Manager::lastErrorText());
    return;
  }
  scheduleDeferredRestart();
  noteHttpResponseSent();
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
  noteWebActivity();
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

  noteHttpResponseSent();
  server.send(200, "text/html", html);
}

static void handleManualUpdateUpload() {
  noteWebActivity();
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
  noteWebActivity();
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
  noteHttpResponseSent();
  server.send(code, contentType, body);
}

// Status endpoint'i kullanici panelinin ana veri kaynagidir.
static void handleStatus() {
  noteWebActivity();
  auto m = pilot_get();
  uint32_t nowMs = millis();

  float rawIa = safeFinite(current_sensor_get_irms_a());
  float rawIb = safeFinite(current_sensor_get_irms_b());
  float rawIc = safeFinite(current_sensor_get_irms_c());
  bool relaySet = relay_get();
  bool chargingState = (m.stateStable == "C" || m.stateStable == "D");
  bool accountingEnabled = relaySet && pwmEnabled && chargingState;
  if (!accountingEnabled) {
    rawIa = 0.0f;
    rawIb = 0.0f;
    rawIc = 0.0f;
  }
  float ia = 0.0f;
  float ib = 0.0f;
  float ic = 0.0f;
  updateDisplayCurrents(rawIa, rawIb, rawIc, &ia, &ib, &ic);
  float pW = accountingEnabled ? safeFinite(g_powerW) : 0.0f;
  float eKWh = safeFinite(g_energyKWh);
  float cpHigh = safeFinite(m.cpHigh);
  float cpLow = safeFinite(m.cpLow);
  float adcHigh = safeFinite(m.adcHigh);
  float adcLow = safeFinite(m.adcLow);
  int ppRaw = analogRead(PP_ADC_PIN);
  float ppVolt = safeFinite((ppRaw * 3.3f) / 4095.0f);
  const char* relayLabel = relaySet ? "SET" : "RESET";
  float iMax = rawIa;
  if (rawIb > iMax) iMax = rawIb;
  if (rawIc > iMax) iMax = rawIc;
  float iAvgSum = 0.0f;
  int iAvgCount = 0;
  if (ia > 0.1f) {
    iAvgSum += ia;
    iAvgCount++;
  }
  if (ib > 0.1f) {
    iAvgSum += ib;
    iAvgCount++;
  }
  if (ic > 0.1f) {
    iAvgSum += ic;
    iAvgCount++;
  }
  float iAvg = (iAvgCount > 0) ? (iAvgSum / (float)iAvgCount) : 0.0f;
  float calA = 0.0f, calB = 0.0f, calC = 0.0f;
  float offA = 0.0f, offB = 0.0f, offC = 0.0f;
  float rngLowMax = 0.0f, rngMidMax = 0.0f, rngLowOff = 0.0f, rngMidOff = 0.0f;
  current_sensor_get_calibration(&calA, &calB, &calC, &offA, &offB, &offC);
  current_sensor_get_range_profile(&rngLowMax, &rngMidMax, &rngLowOff, &rngMidOff);

  String wifiSsid = "-";
  String wifiLoc = "-";
  String ipStr = "-";
  refreshDeviceIdentity();
  String hostStr = String(currentHostName()) + ".local";
  String macStr = s_deviceMac;
  String stationCodeStr = s_stationCode;
  String stationNameStr = s_stationLabel;
  String wifiCfgSsid = s_customWifiSsid;
  String wifiCfgMode = s_wifiUseDhcp ? "dhcp" : "static";
  String wifiCfgIp = ipToString(s_staticIp);
  String wifiCfgGw = ipToString(s_staticGateway);
  String wifiCfgSubnet = ipToString(s_staticSubnet);
  String wifiCfgDns1 = ipToString(s_staticDns1);
  String wifiCfgDns2 = ipToString(s_staticDns2);
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
    "\"cpHigh\":%.2f,\"cpLow\":%.2f,\"adcHigh\":%.3f,\"adcLow\":%.3f,\"ppRaw\":%d,\"ppVolt\":%.3f,"
    "\"stateRaw\":\"%s\",\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,\"iAvg\":%.2f,"
    "\"pW\":%.1f,\"eKWh\":%.3f,\"tSec\":%lu,\"phase\":%d,\"rLbl\":\"%s\","
    "\"wifiSsid\":\"%s\",\"wifiLoc\":\"%s\",\"ip\":\"%s\",\"host\":\"%s\",\"mac\":\"%s\",\"stationCode\":\"%s\",\"stationName\":\"%s\",\"stationAddr\":\"%s\","
    "\"wifiCfgEnabled\":%d,\"wifiCfgSsid\":\"%s\",\"wifiCfgMode\":\"%s\",\"wifiCfgIp\":\"%s\",\"wifiCfgGw\":\"%s\",\"wifiCfgSubnet\":\"%s\",\"wifiCfgDns1\":\"%s\",\"wifiCfgDns2\":\"%s\","
    "\"state\":\"%s\",\"div\":%.3f,\"thb\":%.2f,\"thc\":%.2f,\"thd\":%.2f,\"the\":%.2f,"
    "\"icalA\":%.2f,\"icalB\":%.2f,\"icalC\":%.2f,\"ioffA\":%.2f,\"ioffB\":%.2f,\"ioffC\":%.2f,"
    "\"rngLowMax\":%.2f,\"rngMidMax\":%.2f,\"rngLowOff\":%.2f,\"rngMidOff\":%.2f,"
    "\"modeId\":%d,\"mode\":\"%s\",\"limitA\":%.1f,\"limitTargetA\":%.1f,\"staOk\":%d,"
    "\"mapLat\":%.5f,\"mapLng\":%.5f,\"mapRadius\":%u,"
    "\"otaCur\":\"%s\",\"otaRemote\":\"%s\",\"otaPart\":\"%s\",\"otaImgState\":\"%s\",\"otaStatus\":\"%s\",\"otaErr\":\"%s\",\"otaAgeMs\":%lu,"
    "\"alarmLv\":%d,\"alarmTxt\":\"%s\","
    "\"sLive\":%d,\"sLiveStart\":%lu,\"sLiveSec\":%lu,\"sLiveKWh\":%.3f,"
    "\"rstTotal\":%lu,\"rstNow\":%lu,\"rstHist\":%lu,\"rstLastSec\":%lu,\"rstLastMode\":\"%s\"}",
    loopIntervalMs,
    (unsigned long)relayOnDelayMs,
    (unsigned long)relayOffDelayMs,
    stableCount,
    cpHigh, cpLow, adcHigh, adcLow, ppRaw, ppVolt,
    m.stateRaw.c_str(),
    ia, ib, ic, iAvg,
    pW, eKWh,
    (unsigned long)g_chargeSeconds,
    g_phaseCount,
    relayLabel,
    wifiSsid.c_str(),
    wifiLoc.c_str(),
    ipStr.c_str(),
    hostStr.c_str(),
    macStr.c_str(),
    stationCodeStr.c_str(),
    stationNameStr.c_str(),
    s_stationAddress,
    s_customWifiEnabled ? 1 : 0,
    wifiCfgSsid.c_str(),
    wifiCfgMode.c_str(),
    wifiCfgIp.c_str(),
    wifiCfgGw.c_str(),
    wifiCfgSubnet.c_str(),
    wifiCfgDns1.c_str(),
    wifiCfgDns2.c_str(),
    m.stateStable.c_str(),
    CP_DIVIDER_RATIO,
    TH_B_MIN, TH_C_MIN, TH_D_MIN, TH_E_MIN,
    calA, calB, calC, offA, offB, offC,
    rngLowMax, rngMidMax, rngLowOff, rngMidOff,
    g_chargeMode,
    chargeModeLabel(g_chargeMode),
    safeFinite(g_currentLimitA),
    safeFinite(g_targetCurrentLimitA),
    staOk ? 1 : 0,
    s_mapLat,
    s_mapLng,
    (unsigned)kMapRadiusM,
    otaCurrent,
    otaRemote,
    otaPart,
    otaImgState,
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

  server.send(200, "application/json", s_jsonBuf);
  noteHttpResponseSent();
}

// Gecmis seanslar icin ayri JSON endpoint.
static void handleHistory() {
  noteWebActivity();
  int n = 0;
  n += snprintf(
    s_jsonBuf + n, sizeof(s_jsonBuf) - n,
    "{\"count\":%d,\"active\":{\"on\":%d,\"start\":%lu,\"sec\":%lu,\"kWh\":%.3f},\"items\":[",
    g_histCount,
    g_sessionLive ? 1 : 0,
    (unsigned long)g_sessionLiveStartSec,
    (unsigned long)g_sessionLiveSeconds,
    safeFinite(g_sessionLiveEnergyKWh)
  );

  int start = (g_histCount < 20) ? 0 : g_histHead;
  for (int i = 0; i < g_histCount && n < (int)sizeof(s_jsonBuf) - 2; i++) {
    int idx = (start + i) % 20;
    n += snprintf(
      s_jsonBuf + n, sizeof(s_jsonBuf) - n,
      "%s{\"s\":%lu,\"d\":%lu,\"e\":%.3f,\"p\":%.1f,\"ph\":%u}",
      (i == 0) ? "" : ",",
      (unsigned long)g_histStartSec[idx],
      (unsigned long)g_histDurationSec[idx],
      safeFinite(g_histEnergyKWh[idx]),
      safeFinite(g_histAvgPowerW[idx]),
      (unsigned)g_histPhaseCount[idx]
    );
  }
  snprintf(s_jsonBuf + n, sizeof(s_jsonBuf) - n, "]}");
  server.send(200, "application/json", s_jsonBuf);
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
  float calA = 0.0f, calB = 0.0f, calC = 0.0f;
  float offA = 0.0f, offB = 0.0f, offC = 0.0f;
  float rngLowMax = 0.0f, rngMidMax = 0.0f, rngLowOff = 0.0f, rngMidOff = 0.0f;
  bool locationChanged = false;
  bool stationNameChanged = false;
  bool userCssChanged = false;
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
  if (server.hasArg("limitA")) {
    g_targetCurrentLimitA = clampFloatArg(server.arg("limitA"), 6.0f, 32.0f, g_targetCurrentLimitA);
    saveCurrentLimitSetting();
  }
  if (server.hasArg("div")) {
    pilot_set_divider(clampFloatArg(server.arg("div"), 0.1f, 20.0f, CP_DIVIDER_RATIO));
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
  if (server.hasArg("s") || server.hasArg("thb") || server.hasArg("thc") ||
      server.hasArg("thd") || server.hasArg("the")) {
    pilot_save_state_thresholds();
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
  if (server.hasArg("stationName")) {
    String requested = server.arg("stationName");
    requested.trim();
    if (requested.length() > 0) {
      s_stationLabelCustom = true;
      snprintf(s_stationCustomLabel, sizeof(s_stationCustomLabel), "%s", requested.c_str());
    } else {
      s_stationLabelCustom = false;
      s_stationCustomLabel[0] = '\0';
    }
    refreshDeviceIdentity();
    stationNameChanged = true;
  }
  if (server.hasArg("mapLat")) {
    double candidate = server.arg("mapLat").toDouble();
    if (isValidLatitude(candidate)) {
      s_mapLat = candidate;
      locationChanged = true;
    }
  }
  if (server.hasArg("mapLng")) {
    double candidate = server.arg("mapLng").toDouble();
    if (isValidLongitude(candidate)) {
      s_mapLng = candidate;
      locationChanged = true;
    }
  }
  if (server.hasArg("userCss")) {
    String requested = server.arg("userCss");
    requested.replace("\r", "");
    if (requested.length() > kMaxUserScreenCssLen) {
      requested.remove(kMaxUserScreenCssLen);
    }
    if (requested != s_userScreenCustomCss) {
      s_userScreenCustomCss = requested;
      userCssChanged = true;
    }
  }
  current_sensor_set_calibration(calA, calB, calC, offA, offB, offC);
  current_sensor_set_range_profile(rngLowMax, rngMidMax, rngLowOff, rngMidOff);
  if (locationChanged) {
    refreshStationAddress(true);
  }
  if (locationChanged || stationNameChanged) {
    saveLocationSettings();
  }
  if (userCssChanged) {
    saveUserScreenCssSetting();
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
#if RELAY_USE_LATCH_PINS
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_RESET_PIN);
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
#else
  server.send(404, "text/plain", "Latch cikislari bu kartta devre disi");
  noteHttpResponseSent();
#endif
}

static void handlePulseSet() {
#if RELAY_USE_LATCH_PINS
  if (!requireAdminAuth()) return;
  pulseGpio(MOSFET_SET_PIN);
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
#else
  server.send(404, "text/plain", "Latch cikislari bu kartta devre disi");
  noteHttpResponseSent();
#endif
}


// 4) Route kayitlari ve servis baslatma.
void web_init() {
  // Boot sirasinda ag, OTA, route ve web server bu noktada ayaga kalkar.
  Serial.println("[WEB] web_init start");
  s_resetPrefsReady = s_resetPrefs.begin("evse", false);
  if (s_resetPrefsReady) {
    loadResetStats();
  } else {
    Serial.println("[RST] NVS init fail");
  }
  loadWifiSettings();
  loadCurrentLimitSetting();
  loadLocationSettings();
  loadUserScreenCssSetting();
  setupWiFi();
  setupArduinoOta();
#if RELAY_USE_LATCH_PINS
  pinMode(MOSFET_RESET_PIN, OUTPUT);
  pinMode(MOSFET_SET_PIN, OUTPUT);
  digitalWrite(MOSFET_RESET_PIN, LOW);
  digitalWrite(MOSFET_SET_PIN, LOW);
#endif
  // HTTP route kayitlari.
  server.on("/", HTTP_GET, handleRoot);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/settings", HTTP_GET, handleCalibrationPage);
  server.on("/calibration", HTTP_GET, handleCalibrationPage);
  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/update", HTTP_GET, handleManualUpdatePage);
  server.on("/update", HTTP_POST, handleManualUpdateResult, handleManualUpdateUpload);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/sw.js", HTTP_GET, handleServiceWorker);
  server.on("/app-icon.svg", HTTP_GET, handleAppIcon);
  server.on("/vehicle-top-art.svg", HTTP_GET, handleVehicleTopArt);
  server.on("/ota_check", HTTP_GET, handleOtaCheck);
  server.on("/ota_install", HTTP_GET, handleOtaInstall);
  server.on("/wifi_scan", HTTP_GET, handleWifiScan);
  server.on("/wifi_apply", HTTP_GET, handleWifiApply);
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
#if RELAY_USE_LATCH_PINS
  server.on("/pulse_reset", HTTP_GET, handlePulseReset);
  server.on("/pulse_set", HTTP_GET, handlePulseSet);
#endif
  server.onNotFound(handleRoot);
  s_serverStarted = false;
  ensureServerStarted();
  if (s_webTaskHandle == nullptr) {
    BaseType_t taskOk = xTaskCreatePinnedToCore(
      web_task_runner,
      "webLoop",
      6144,
      nullptr,
      1,
      &s_webTaskHandle,
      1
    );
    if (taskOk == pdPASS) {
      Serial.println("[WEB] web task started");
    } else {
      s_webTaskHandle = nullptr;
      Serial.println("[WEB] web task start FAILED");
    }
  }
  Serial.println("[WEB] web_init done");
}

static void web_tick() {
  // Arka plan servisleri her loop'ta buradan yurutulur.
  ensureServerStarted();
  if (!s_serverStarted) return;
  server.handleClient();

  if (s_manualOtaRebootPending && (int32_t)(millis() - s_manualOtaRebootAtMs) >= 0) {
    s_manualOtaRebootPending = false;
    Serial.println("[BOOTCTL] Web komutu sonrasi yeniden baslatiliyor");
    delay(100);
    ESP.restart();
  }

  // WiFi yeniden baglanti denemesi: kisa timeout ile tekrar dene, uzun tarama ile donguyu kilitleme.
  static uint32_t lastWifiTryMs = 0;
  const uint32_t nowTry = millis();
  if (WiFi.status() != WL_CONNECTED && (nowTry - lastWifiTryMs >= 10000)) {
    lastWifiTryMs = nowTry;
    if (s_customWifiEnabled && s_customWifiSsid.length() > 0) {
      applyIpMode();
      WiFi.begin(s_customWifiSsid.c_str(), s_customWifiPassword.c_str());
    } else {
      wifiMulti.run(1000);
    }
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
      Serial.print(currentHostName());
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

static void web_task_runner(void* /*arg*/) {
  for (;;) {
    web_tick();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void web_loop() {
  if (s_webTaskHandle != nullptr) return;
  web_tick();
}

bool web_ready_for_ota_validation() {
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
  if (!staOk) return false;
  if (s_successfulHttpResponses == 0) return false;
  return s_lastHttpRequestMs != 0;
}
