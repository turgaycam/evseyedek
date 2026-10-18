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
#include "app_log.h"
#include "OTA_Manager.h"
#include "vehicle_top_art_svg.h"
#include "togg_arac_v2_webp.h"
#include "pilot/pilot.h"
#include "io/relay.h"

#include "io/current_sensor.h"
#include "auth.h"
#include "cloud_client.h"
#include "schedule.h"

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
#error "EVSE_ADMIN_PASSWORD secrets.ini icinde tanimlanmali (bkz. secrets.example.ini)"
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
#define EVSE_WIFI_1_SSID ""
#endif
#ifndef EVSE_WIFI_1_PASS
#define EVSE_WIFI_1_PASS ""
#endif

#ifndef EVSE_WIFI_2_LOC
#define EVSE_WIFI_2_LOC "Rotosis"
#endif
#ifndef EVSE_WIFI_2_SSID
#define EVSE_WIFI_2_SSID ""
#endif
#ifndef EVSE_WIFI_2_PASS
#define EVSE_WIFI_2_PASS ""
#endif

#ifndef EVSE_WIFI_3_LOC
#define EVSE_WIFI_3_LOC "Ceylan Robot"
#endif
#ifndef EVSE_WIFI_3_SSID
#define EVSE_WIFI_3_SSID ""
#endif
#ifndef EVSE_WIFI_3_PASS
#define EVSE_WIFI_3_PASS ""
#endif

#ifndef EVSE_WIFI_4_LOC
#define EVSE_WIFI_4_LOC "Rotosis Atolye"
#endif
#ifndef EVSE_WIFI_4_SSID
#define EVSE_WIFI_4_SSID ""
#endif
#ifndef EVSE_WIFI_4_PASS
#define EVSE_WIFI_4_PASS ""
#endif

#ifndef EVSE_WIFI_5_LOC
#define EVSE_WIFI_5_LOC "Test"
#endif
#ifndef EVSE_WIFI_5_SSID
#define EVSE_WIFI_5_SSID ""
#endif
#ifndef EVSE_WIFI_5_PASS
#define EVSE_WIFI_5_PASS ""
#endif

// Panel kullanici adi/parolasi artik auth.cpp'de (NVS'de tuzlu ozet). EVSE_ADMIN_PASSWORD
// yalnizca ilk acilistaki varsayilan paroladir.
static const char* kHostNameBase = EVSE_HOSTNAME;
static const char* kStationName = "Rotosis Robotlu Otomasyon";
static const char* kDefaultStationAddress = "Fevzi Cakmak Mah. Sehit Ibrahim Betin Cd. No:4/F, Arli Sanayi Sitesi, Karatay / Konya";
static constexpr double kDefaultMapLat = 37.94559;
static constexpr double kDefaultMapLng = 32.58082;
static constexpr uint16_t kMapRadiusM = 520;
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
static char s_jsonBuf[5120];  // /status + bulut + planli sarj alanlari
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
  bool authorized = false;  // bu yukleme gecerli bilet/jetonla mi basladi
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

// Kullanici CSS duzenleyici kaldirildi, fakat bu karttaki eski NVS kaydi silinmez.
static void purgeLegacyUserCss() {
  (void)s_resetPrefsReady;
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
  s_manualOta.authorized = false;
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

// Yonetim veri uclari "Authorization: Bearer <jeton>" ister (bkz. auth.cpp).
// Jeton yoksa / gecersizse 401 JSON doner; tarayici Basic Auth penceresi acilmaz.
static bool requireAdminAuth() {
  server.sendHeader("Cache-Control", "no-store");
  if (auth_check_bearer(server.header("Authorization"))) {
    return true;
  }
  server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
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
<!DOCTYPE html><html lang="tr"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="color-scheme" content="light dark">
<meta name="theme-color" content="#0c425e">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="RTevCharge"><meta name="format-detection" content="telephone=no">
<title>RTevCharge</title>
<link rel="manifest" href="/manifest.json">
<link rel="icon" href="/app-icon.svg" type="image/svg+xml">
<style>
:root{--lacivert:#0c425e;--turuncu:#ff5a1f;--mavi:#00b2ff;--zemin:#f3f6f8;--yuzey:#fff;--kenar:#d5dee5;--metin:#14212b;--soluk:#4d5f6d;--iyi:#11703f;--iyi-z:#e2f5ea;--kotu:#b3261e;--kotu-z:#fbe6e4;--uyari:#8a5a00;--uyari-z:#fff3d6;--bilgi:#00618c;--bilgi-z:#e0f3fc;--dugme:#0c425e;--dugme-metin:#fff;--odak:#ff5a1f;--kod:#f0f3f6;--arac:#1f5f80;--st:env(safe-area-inset-top);--sb:env(safe-area-inset-bottom);--sl:env(safe-area-inset-left);--sr:env(safe-area-inset-right)}
@media(prefers-color-scheme:dark){:root{--zemin:#0b1a24;--yuzey:#11283a;--kenar:#24435a;--metin:#e7eef3;--soluk:#a9bccb;--iyi:#5fd896;--iyi-z:#123526;--kotu:#ff8a80;--kotu-z:#3a1a1a;--uyari:#ffc760;--uyari-z:#3a2c0e;--bilgi:#6fd3ff;--bilgi-z:#0c3047;--dugme:#00b2ff;--dugme-metin:#06141d;--kod:#0a1720;--arac:#4a8db3}}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html{-webkit-text-size-adjust:100%;font-size:16px}
@supports(font:-apple-system-body){html{font:-apple-system-body}}
body{margin:0;background:var(--zemin);color:var(--metin);font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;font-size:1rem;line-height:1.45;overscroll-behavior-y:none}
[hidden]{display:none!important}
:focus-visible{outline:3px solid var(--odak);outline-offset:2px}
a,button{touch-action:manipulation}
.dugme,button{display:inline-flex;align-items:center;justify-content:center;gap:8px;min-height:48px;padding:10px 18px;border-radius:10px;border:1px solid transparent;background:var(--dugme);color:var(--dugme-metin);font:inherit;font-weight:700;text-decoration:none;cursor:pointer}
.dugme.ik,button.ik{background:transparent;color:var(--metin);border-color:var(--kenar)}
svg.i{width:1.5em;height:1.5em;flex:none}
/* ust bar */
header{background:var(--lacivert);color:#fff;border-bottom:3px solid var(--turuncu);padding:var(--st) var(--sr) 0 var(--sl)}
.ust{display:flex;align-items:center;gap:12px;padding:10px 16px;min-height:60px;max-width:560px;margin:0 auto}
.lg{width:36px;height:36px;flex:none}
.ist{flex:1;min-width:0;line-height:1.25}
.ist b{display:block;font-size:1.05rem;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ist small{display:block;font-size:.8rem;opacity:.85;overflow-wrap:anywhere}
.canli{display:inline-flex;align-items:center;gap:6px;font-size:.75rem;font-weight:700;letter-spacing:.04em;padding:4px 10px;border-radius:999px;border:1px solid rgba(255,255,255,.5);flex:none}
.canli::before{content:"";width:8px;height:8px;border-radius:50%;background:#8aa3b3}
.canli.on::before{background:#5fd896}
#netStrip{padding:10px calc(16px + var(--sr)) 10px calc(16px + var(--sl));font-size:.9rem;font-weight:600;text-align:center}
#netStrip.uyari{background:var(--uyari-z);color:var(--uyari)}#netStrip.kotu{background:var(--kotu-z);color:var(--kotu)}
main{max-width:560px;margin:0 auto;padding:16px calc(16px + var(--sr)) calc(24px + var(--sb)) calc(16px + var(--sl));display:grid;gap:14px}
.kart{background:var(--yuzey);border:1px solid var(--kenar);border-radius:14px;padding:16px;min-width:0}
.kart h2{margin:0 0 10px;font-size:.78rem;text-transform:uppercase;letter-spacing:.06em;color:var(--soluk);display:flex;align-items:center;gap:8px}
.kart h2 .s{margin-left:auto;text-transform:none;letter-spacing:0;font-weight:600}
.alarm{padding:12px 14px;border-radius:10px;font-weight:600;border-left:5px solid;display:flex;gap:10px;align-items:flex-start}
.alarm.l1{background:var(--uyari-z);color:var(--uyari)}.alarm.l2{background:var(--kotu-z);color:var(--kotu)}
/* durum karti: renk data-s ile */
.durum{display:flex;align-items:center;gap:16px;padding:20px 18px;border-left:8px solid var(--soluk);--r:var(--soluk);--rz:var(--kod)}
.durum[data-s=A]{--r:var(--soluk);--rz:var(--kod)}
.durum[data-s=B]{--r:var(--bilgi);--rz:var(--bilgi-z)}
.durum[data-s=C],.durum[data-s=D]{--r:var(--iyi);--rz:var(--iyi-z)}
.durum[data-s=E],.durum[data-s=F]{--r:var(--kotu);--rz:var(--kotu-z)}
.durum{border-left-color:var(--r);flex-wrap:wrap}
/* arac gorseli */
.arac{flex:1 1 100%;width:100%;height:auto;max-height:200px}
.arac .zemin{stroke:var(--kenar);stroke-width:2}
.arac .post{fill:var(--lacivert);stroke:var(--mavi);stroke-width:1.5}
.arac .ekr{fill:var(--mavi)}
.arac .tepe{fill:var(--turuncu)}
.arac .post,.arac .ekr,.arac .tepe{transition:fill .4s,stroke .4s}
:is([data-s=C],[data-s=D]) .arac .post{fill:#12803f;stroke:#5fd896}:is([data-s=C],[data-s=D]) .arac .ekr{fill:#c8f7dc}:is([data-s=C],[data-s=D]) .arac .tepe{fill:#5fd896}
.arac .kab{fill:none;stroke:var(--soluk);stroke-width:1.7;stroke-linecap:round}
.arac .fis{fill:var(--soluk)}
.kaporta{fill:var(--arac);stroke:var(--arac);stroke-width:2;stroke-linejoin:round}
.cam{fill:var(--zemin);opacity:.8}
.ter{fill:#1b2730}.jant{fill:var(--kenar)}
.port{fill:var(--soluk)}
.etk{font-size:8px;font-weight:700;letter-spacing:.12em;fill:var(--soluk)}
.akim{fill:none;stroke:#d6ffe6;stroke-width:2.4;stroke-linecap:round;stroke-dasharray:7 93;filter:drop-shadow(0 0 1.6px #8cffc0)}
.parilti{fill:var(--iyi)}
.batt rect{fill:none;stroke:var(--iyi);stroke-width:2}.batt .bc{fill:var(--iyi)}
.batt .bdol{fill:var(--iyi);stroke:none;transform-box:fill-box;transform-origin:left}
.uyar path{fill:var(--kotu-z);stroke:var(--kotu);stroke-width:2;stroke-linejoin:round}.uyar .un{stroke-linecap:round}
#aracGorsel,.kab,.fis,.akim,.parilti,.batt,.uyar,.kaporta,.cam,.port{transition:opacity .4s,fill .4s,stroke .4s}
.kB,.akim,.parilti,.batt,.uyar{opacity:0}
[data-s=A] #aracGorsel{opacity:.45}
[data-s=A] .kaporta{fill:none;stroke:var(--soluk);stroke-dasharray:5 4}
[data-s=A] .cam{opacity:0}
.durum:not([data-s=A]) .kA{opacity:0}
.durum:not([data-s=A]) .kB{opacity:1}
[data-s=B] .kB{stroke:var(--mavi)}[data-s=B] .port{fill:var(--mavi)}
:is([data-s=C],[data-s=D]) .kB{stroke:var(--iyi)}
:is([data-s=C],[data-s=D]) .port{fill:var(--iyi);animation:nb 1.2s infinite}
[data-s=B] .akim{opacity:1;stroke:#d2f4ff;filter:drop-shadow(0 0 1.6px #5fd0ff);animation:ak2 2.6s linear infinite}
:is([data-s=C],[data-s=D]) .akim{opacity:1;animation:ak2 1.3s linear infinite}
:is([data-s=C],[data-s=D]) .parilti{opacity:.3;animation:nb 2.4s infinite}
:is([data-s=C],[data-s=D]) .batt{opacity:1}
:is([data-s=C],[data-s=D]) .bdol{animation:bd 3s ease-in-out infinite}
:is([data-s=E],[data-s=F]) .kaporta{stroke:var(--kotu)}
:is([data-s=E],[data-s=F]) .kB{stroke:var(--kotu)}
:is([data-s=E],[data-s=F]) .port{fill:var(--kotu)}
:is([data-s=E],[data-s=F]) .uyar{opacity:1;animation:nb 1s infinite}
.sw{opacity:0;mix-blend-mode:screen}
.swp{transform:translateX(-90px)}
:is([data-s=C],[data-s=D]) .sw{opacity:.95}
:is([data-s=C],[data-s=D]) .swp{animation:sw 2.2s ease-in-out infinite}
:is([data-s=C],[data-s=D]) #aracGorsel{filter:drop-shadow(0 0 3px rgba(40,230,140,.85))}
@keyframes sw{0%{transform:translateX(-90px)}100%{transform:translateX(250px)}}
.swB,.swR{transition:opacity .4s;opacity:0;mix-blend-mode:screen}
.durum[data-s=B] .swB{opacity:.9}
:is(.durum[data-s=E],.durum[data-s=F]) .swR{opacity:.9}
.durum[data-s=B] #aracGorsel{filter:drop-shadow(0 0 3px rgba(0,178,255,.9))}
:is(.durum[data-s=E],.durum[data-s=F]) #aracGorsel{filter:drop-shadow(0 0 3px rgba(255,60,50,.9))}
.durum[data-s=B] .swB{animation:nfB 3.2s ease-in-out infinite}
:is(.durum[data-s=E],.durum[data-s=F]) .swR{animation:nfR 1.6s ease-in-out infinite}
.durum[data-s=B] #aracGorsel{animation:hlB 3.2s ease-in-out infinite}
:is(.durum[data-s=E],.durum[data-s=F]) #aracGorsel{animation:hlR 1.6s ease-in-out infinite}
@keyframes nfB{0%,100%{opacity:.35}50%{opacity:1}}
@keyframes nfR{0%,100%{opacity:.4}50%{opacity:1}}
@keyframes hlB{0%,100%{filter:drop-shadow(0 0 1px rgba(0,178,255,.4))}50%{filter:drop-shadow(0 0 5px rgba(0,178,255,1))}}
@keyframes hlR{0%,100%{filter:drop-shadow(0 0 1px rgba(255,60,50,.4))}50%{filter:drop-shadow(0 0 5px rgba(255,60,50,1))}}
@keyframes ak{to{stroke-dashoffset:-24}}
@keyframes ak2{from{stroke-dashoffset:0}to{stroke-dashoffset:100}}
@keyframes bd{0%{transform:scaleX(.1)}85%,100%{transform:scaleX(1)}}
.dIk{width:72px;height:72px;border-radius:18px;display:grid;place-items:center;background:var(--rz);color:var(--r);flex:none}
.dIk svg{width:44px;height:44px}
.durum[data-s=C] .dIk svg,.durum[data-s=D] .dIk svg{animation:nb 1.6s ease-in-out infinite}
@keyframes nb{50%{opacity:.45}}
.dT{font-size:1.45rem;font-weight:800;line-height:1.2;margin:0}
.dA{color:var(--soluk);font-size:.95rem;margin:4px 0 0}
/* canli sarj */
.guc{font-size:3.4rem;font-weight:800;line-height:1;font-variant-numeric:tabular-nums;margin:4px 0 14px;overflow-wrap:anywhere}
.guc u,.kv u{text-decoration:none;font-size:1rem;font-weight:600;color:var(--soluk);margin-left:4px}
.kvs{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.kv{background:var(--kod);border-radius:10px;padding:10px 12px}
.kv small{display:block;font-size:.78rem;color:var(--soluk);font-weight:600}
.kv b{font-size:1.35rem;font-variant-numeric:tabular-nums}
.ph{display:grid;grid-template-columns:2.2em 1fr 4.6em;align-items:center;gap:10px;margin-top:12px}
.ph span{font-size:.8rem;font-weight:700;color:var(--soluk)}
.ph b{text-align:right;font-variant-numeric:tabular-nums}
.mt{height:12px;border-radius:9px;background:var(--kod);overflow:hidden}
.mt i{display:block;height:100%;width:0;background:var(--mavi);transition:width .6s}
.lim{margin:14px 0 0;font-size:.88rem;color:var(--soluk)}
/* QR yonlendirme */
.qr{display:flex;gap:16px;align-items:center;border-top:4px solid var(--turuncu)}
.qr svg.q{width:96px;height:96px;flex:none;color:var(--lacivert)}
@media(prefers-color-scheme:dark){.qr svg.q{color:var(--mavi)}}
.qr p{margin:0 0 12px;font-weight:600}
.qr .dugme{width:100%}
@media(max-width:360px){.qr{flex-direction:column;text-align:center}}
dl{display:grid;grid-template-columns:1fr auto;gap:8px 12px;margin:0}
dt{color:var(--soluk)}dd{margin:0;text-align:right;font-weight:700;font-variant-numeric:tabular-nums}
.bos{color:var(--soluk);font-size:.92rem;margin:0;display:flex;gap:10px;align-items:center}
footer{text-align:center;font-size:.82rem;color:var(--soluk);display:grid;gap:8px;justify-items:center}
footer a{display:inline-flex;align-items:center;min-height:44px;padding:0 12px;color:var(--soluk)}
@media(prefers-reduced-motion:reduce){*{transition:none!important;animation:none!important}}
</style></head><body>
<svg width="0" height="0" style="position:absolute" aria-hidden="true"><defs>
<symbol id="lg" viewBox="0 0 64 64"><rect width="64" height="64" rx="12" fill="#0c425e"/><g transform="translate(5 4.4) scale(.6)"><path d="M69.93 88.81C69.64 88.50 69.21 87.87 68.97 87.41C68.03 85.58 67.66 84.92 67.34 84.45C67.15 84.18 67.00 83.89 67.00 83.81C67.00 83.72 66.80 83.36 66.55 83.00C66.30 82.64 66.04 82.17 65.97 81.96C65.89 81.74 65.71 81.39 65.55 81.19C65.40 80.98 65.15 80.53 65.00 80.19C64.85 79.84 64.63 79.45 64.50 79.31C64.37 79.17 64.15 78.79 64.01 78.46C63.87 78.13 63.62 77.68 63.46 77.46C63.30 77.24 63.09 76.87 63.00 76.62C62.90 76.38 62.70 76.02 62.55 75.81C62.39 75.61 62.15 75.16 62.00 74.81C61.85 74.47 61.63 74.08 61.50 73.94C61.37 73.80 61.15 73.41 61.00 73.06C60.85 72.72 60.61 72.27 60.45 72.06C60.30 71.86 60.10 71.49 60.00 71.25C59.91 71.01 59.70 70.63 59.54 70.41C59.38 70.19 59.13 69.74 58.99 69.41C58.85 69.08 58.63 68.70 58.50 68.56C58.37 68.42 58.15 68.04 58.01 67.71C57.87 67.38 57.62 66.93 57.46 66.71C57.29 66.49 57.10 66.14 57.03 65.92C56.96 65.70 56.70 65.23 56.45 64.87C56.20 64.52 56.00 64.15 56.00 64.06C56.00 63.97 55.81 63.63 55.59 63.30C55.36 62.96 55.10 62.49 55.00 62.25C54.90 62.01 54.64 61.54 54.41 61.20C54.19 60.87 54.00 60.53 54.00 60.45C54.00 60.38 53.77 59.98 53.50 59.58C53.22 59.17 53.00 58.77 53.00 58.68C53.00 58.60 52.81 58.25 52.59 57.92C52.36 57.59 52.10 57.12 52.00 56.88C51.90 56.63 51.64 56.16 51.41 55.83C51.19 55.50 51.00 55.15 51.00 55.07C51.00 54.98 50.85 54.69 50.66 54.42C50.48 54.16 50.15 53.60 49.93 53.19C49.71 52.77 49.32 52.04 49.07 51.56C48.81 51.08 48.53 50.60 48.44 50.50C48.34 50.40 48.15 50.04 48.01 49.70C47.86 49.37 47.61 48.93 47.45 48.74C47.28 48.54 47.08 48.17 46.99 47.91C46.91 47.65 46.71 47.27 46.55 47.06C46.40 46.86 46.15 46.41 46.01 46.07C45.86 45.74 45.58 45.27 45.37 45.04C44.55 44.10 44.97 43.41 46.60 43.07C47.13 42.96 48.01 42.70 48.56 42.50C49.11 42.30 49.95 42.04 50.41 41.94C50.88 41.84 51.53 41.64 51.85 41.50C52.17 41.37 52.94 41.14 53.55 41.00C54.17 40.86 54.93 40.63 55.24 40.49C55.55 40.35 56.29 40.13 56.87 40.00C57.46 39.87 58.27 39.62 58.69 39.46C59.10 39.29 59.77 39.09 60.19 39.01C60.60 38.93 61.27 38.73 61.69 38.56C62.10 38.39 62.92 38.15 63.50 38.01C64.78 37.71 66.88 36.95 67.02 36.72C67.61 35.81 67.87 33.73 67.87 29.88C67.87 26.38 67.81 25.82 67.20 23.52C66.62 21.32 64.81 19.46 62.75 18.94C62.30 18.83 61.57 18.62 61.13 18.49C59.59 18.02 46.90 17.57 29.94 17.37C1.55 17.05 2.44 17.08 2.32 16.45C2.24 16.08 2.13 11.92 2.11 8.90C2.09 6.72 2.12 6.33 2.29 6.16C2.44 6.00 3.00 5.92 4.65 5.79C5.84 5.70 7.57 5.54 8.50 5.44C9.43 5.34 12.47 5.14 15.25 5.00C18.04 4.86 21.52 4.63 23.00 4.50C24.48 4.37 27.71 4.17 30.19 4.07C36.51 3.80 39.37 3.64 41.75 3.44C49.21 2.80 62.24 2.63 66.19 3.12C68.66 3.43 73.36 4.83 74.22 5.51C74.40 5.64 74.79 5.87 75.10 6.00C75.66 6.25 75.88 6.41 77.20 7.54C78.19 8.37 79.77 10.17 80.10 10.82C80.24 11.10 80.44 11.40 80.55 11.49C80.65 11.58 80.85 11.95 80.99 12.32C81.13 12.70 81.39 13.21 81.55 13.47C81.72 13.73 81.92 14.20 81.99 14.51C82.06 14.83 82.28 15.50 82.49 16.01C82.69 16.52 82.94 17.47 83.05 18.12C83.16 18.78 83.35 19.75 83.49 20.28C83.93 21.99 84.06 24.12 84.05 29.69C84.05 35.34 83.96 36.71 83.44 38.50C83.27 39.08 83.07 39.92 83.00 40.37C82.92 40.81 82.73 41.43 82.57 41.74C82.41 42.06 82.16 42.69 82.01 43.15C81.86 43.61 81.63 44.12 81.51 44.27C81.39 44.43 81.16 44.84 81.01 45.19C80.86 45.53 80.64 45.92 80.51 46.06C80.39 46.20 80.17 46.53 80.02 46.80C79.53 47.65 76.55 50.88 76.24 50.88C76.19 50.88 75.83 51.13 75.43 51.44C75.03 51.75 74.64 52.00 74.55 52.00C74.47 52.00 74.13 52.19 73.80 52.42C73.46 52.65 72.96 52.91 72.69 52.99C72.41 53.08 71.93 53.31 71.62 53.50C71.32 53.70 70.81 53.92 70.50 54.00C68.94 54.41 68.50 55.39 69.44 56.38C69.60 56.55 69.85 56.96 69.99 57.29C70.13 57.62 70.38 58.07 70.54 58.29C70.71 58.51 70.90 58.86 70.97 59.08C71.04 59.30 71.30 59.77 71.55 60.13C71.80 60.48 72.00 60.85 72.00 60.93C72.00 61.02 72.15 61.31 72.34 61.58C72.52 61.84 72.85 62.40 73.07 62.81C74.16 64.87 74.36 65.22 74.67 65.67C74.85 65.94 75.00 66.23 75.00 66.32C75.00 66.40 75.19 66.75 75.41 67.08C75.64 67.41 75.90 67.88 76.00 68.12C76.09 68.37 76.30 68.73 76.45 68.94C76.61 69.14 76.85 69.59 77.00 69.94C77.15 70.28 77.37 70.67 77.50 70.81C77.63 70.95 77.85 71.34 78.00 71.69C78.15 72.03 78.35 72.40 78.45 72.51C78.61 72.69 78.89 73.20 79.94 75.25C80.14 75.65 80.41 76.08 80.52 76.21C80.64 76.33 80.85 76.71 80.99 77.04C81.13 77.37 81.38 77.82 81.54 78.04C81.70 78.26 81.91 78.63 82.00 78.88C82.10 79.12 82.36 79.59 82.59 79.92C82.81 80.25 83.00 80.60 83.00 80.68C83.00 80.77 83.15 81.06 83.34 81.33C83.67 81.80 84.05 82.48 84.91 84.12C85.14 84.57 85.48 85.16 85.67 85.42C85.85 85.69 86.00 85.98 86.00 86.07C86.00 86.15 86.20 86.53 86.45 86.89C87.37 88.26 87.57 88.89 87.16 89.10C87.04 89.17 83.23 89.25 78.70 89.30L70.46 89.38L69.93 88.81Z" fill="#fff" fill-rule="evenodd"/><rect x="18" y="43.4" width="15.4" height="29" fill="#00b2ff"/></g></symbol>
<symbol id="sA" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M9 3v4M15 3v4M7 7h10v4a5 5 0 0 1-10 0z"/><path d="M12 16v5" stroke-dasharray="2 2"/></g></symbol>
<symbol id="sB" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M9 3v4M15 3v4M7 7h10v4a5 5 0 0 1-10 0z"/><path d="M12 16v5"/><path d="m16 19 2 2 4-4"/></g></symbol>
<symbol id="sC" viewBox="0 0 24 24"><path d="M13 2 4 14h7l-1 8 9-12h-7z" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></symbol>
<symbol id="sE" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3 2 20h20z"/><path d="M12 10v4M12 17v.5"/></g></symbol>
<symbol id="iK" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V8a4 4 0 0 1 8 0v3"/></g></symbol>
</defs></svg>

<header><div class="ust">
  <svg class="lg" role="img" aria-label="RTevCharge"><use href="#lg"/></svg>
  <div class="ist"><b id="stationName">Şarj istasyonu</b><small id="stationAddr">-</small></div>
  <span class="canli" id="sync" role="status">BEKLİYOR</span>
</div></header>
<div id="netStrip" role="status" aria-live="polite" hidden></div>

<main>
  <div class="alarm l1" id="alarmBand" role="alert" hidden><svg class="i"><use href="#sE"/></svg><span id="alarmTxt"></span></div>

  <section class="kart durum" id="stateCard" data-s="A" aria-live="polite">
    <!-- Arac gorseli: /togg-arac-v2.webp (firmware icinde PROGMEM). Sahne PWA 1.1.5 ile ayni: direk sagda, ince kablo + akan isik, sarjda yesil direk. -->
    <svg class="arac" id="aracSvg" viewBox="0 0 320 122" role="img" aria-label="Araç görseli: araç bağlı değil"><defs><filter id="yF" color-interpolation-filters="sRGB"><feColorMatrix type="matrix" values="0 0 0 0 .25  0 0 0 0 1  0 0 0 0 .55  0 0 0 1 0"/></filter><linearGradient id="sG" x1="0" x2="1"><stop offset="0" stop-color="#000"/><stop offset=".5" stop-color="#fff"/><stop offset="1" stop-color="#000"/></linearGradient><filter id="yB" color-interpolation-filters="sRGB"><feColorMatrix type="matrix" values="0 0 0 0 0  0 0 0 0 .7  0 0 0 0 1  0 0 0 1 0"/></filter><filter id="yR" color-interpolation-filters="sRGB"><feColorMatrix type="matrix" values="0 0 0 0 1  0 0 0 0 .22  0 0 0 0 .2  0 0 0 1 0"/></filter><mask id="sS" maskUnits="userSpaceOnUse" x="-10" y="-10" width="340" height="130"><rect x="2" y="-10" width="218" height="130" fill="url(#sG)"/></mask><mask id="sM" maskUnits="userSpaceOnUse" x="-10" y="-10" width="340" height="130"><rect class="swp" x="90" y="-10" width="90" height="130" fill="url(#sG)"/></mask></defs>
      <ellipse class="parilti" cx="112" cy="106" rx="104" ry="6"/>
      <path class="zemin" d="M4 105h312"/>
      <rect class="post" x="286" y="26" width="24" height="79" rx="4"/><rect class="ekr" x="291" y="32" width="14" height="9" rx="1.5"/><rect class="tepe" x="286" y="22" width="24" height="5" rx="2"/>
      <path class="kab kA" d="M286 62c-5 24 -10 32 -18 32"/><rect class="kA fis" x="262" y="91" width="9" height="7" rx="2"/>
      <g id="aracGorsel"><image href="/togg-arac-v2.webp" x="2" y="19.3" width="218" height="88.7" preserveAspectRatio="xMidYMax meet"/><circle class="port" cx="168.2" cy="67.5" r="3.5"/></g>
      <g class="sw" mask="url(#sM)"><image filter="url(#yF)" href="/togg-arac-v2.webp" x="2" y="19.3" width="218" height="88.7" preserveAspectRatio="xMidYMax meet"/></g>
      <g class="swB" mask="url(#sS)"><image filter="url(#yB)" href="/togg-arac-v2.webp" x="2" y="19.3" width="218" height="88.7" preserveAspectRatio="xMidYMax meet"/></g>
      <g class="swR" mask="url(#sS)"><image filter="url(#yR)" href="/togg-arac-v2.webp" x="2" y="19.3" width="218" height="88.7" preserveAspectRatio="xMidYMax meet"/></g>
      <path class="kab kB" d="M168.2 69.5C178 70 190 86 190 104C190 112 250 114 286 62"/>
      <path class="akim" pathLength="100" d="M168.2 69.5C178 70 190 86 190 104C190 112 250 114 286 62"/>
      <g class="batt" transform="translate(-270 0)"><rect x="276" y="6" width="30" height="15" rx="3"/><rect x="306" y="10" width="3" height="7" rx="1" class="bc"/><rect class="bdol" x="279" y="9" width="24" height="9" rx="1.5"/></g>
      <g class="uyar" transform="translate(-270 0)"><path d="M292 4l14 24h-28z"/><path d="M292 12v8M292 23v1" class="un"/></g>
    </svg>
    <div class="dIk" aria-hidden="true"><svg><use id="stateIcon" href="#sA"/></svg></div>
    <div><p class="dT" id="stateTitle">Araç bağlı değil</p><p class="dA" id="stateHint">Şarj kablosunu aracınıza takın.</p><p class="dA" id="schedLine" hidden></p></div>
  </section>

  <section class="kart" id="liveCard" hidden>
    <h2>Anlık güç<span class="s" id="phaseInfo">3 faz</span></h2>
    <div class="guc"><span id="pwr">0,0</span><u>kW</u></div>
    <div class="kvs">
      <div class="kv"><small>Enerji</small><b id="ekwh">0,00</b><u>kWh</u></div>
      <div class="kv"><small>Süre</small><b id="tsec">0 dk</b></div>
    </div>
    <div class="ph" id="ph1"><span>L1</span><div class="mt"><i id="b1"></i></div><b id="i1">0,0 A</b></div>
    <div class="ph" id="ph2"><span>L2</span><div class="mt"><i id="b2"></i></div><b id="i2">0,0 A</b></div>
    <div class="ph" id="ph3"><span>L3</span><div class="mt"><i id="b3"></i></div><b id="i3">0,0 A</b></div>
    <p class="lim">Akım limiti: <b id="limitA">32</b> A</p>
  </section>

  <section class="kart qr" id="qrCard">
    <svg class="q" viewBox="0 0 96 96" aria-hidden="true"><g fill="none" stroke="currentColor" stroke-width="5" stroke-linecap="round"><path d="M6 26V6h20M70 6h20v20M90 70v20H70M26 90H6V70"/></g><g fill="currentColor" opacity=".8"><rect x="24" y="24" width="18" height="18" rx="2"/><rect x="54" y="24" width="18" height="18" rx="2"/><rect x="24" y="54" width="18" height="18" rx="2"/><rect x="56" y="56" width="6" height="6"/><rect x="66" y="56" width="6" height="6"/><rect x="56" y="66" width="6" height="6"/><rect x="66" y="66" width="6" height="6"/></g></svg>
    <div><p id="qrText">Şarjı başlatmak için RTevCharge uygulamasından istasyondaki QR kodu okutun.</p>
    <a class="dugme" id="appBtn" href="#" data-todo="uygulama-baglantisi">Uygulamayı aç</a></div>
  </section>

  <section class="kart">
    <h2 id="sessTitle">Bu seans</h2>
    <dl>
      <dt>Süre</dt><dd id="sTime">—</dd>
      <dt>Enerji</dt><dd id="sKwh">—</dd>
      <dt>Tahmini maliyet</dt><dd id="sCost">—</dd>
    </dl>
  </section>

  <section class="kart">
    <h2>Son seanslar</h2>
    <p class="bos"><svg class="i" aria-hidden="true"><use href="#iK"/></svg>Geçmiş seanslarınız, uygulamada giriş yaptığınızda burada görünecek.</p>
  </section>

  <footer>
    <button class="ik" id="installBtn" hidden>Ana ekrana ekle</button>
    <span id="ts">Son güncelleme: -</span>
    <a href="/admin">Yönetici girişi</a>
  </footer>
</main>

<script>
const POLL_MS=3000;
const ST={
  A:{t:"Araç bağlı değil",h:"Şarj kablosunu aracınıza takın.",i:"sA"},
  B:{t:"Araç bağlı, hazır",h:"Şarjı başlatmak için uygulamadan QR kodu okutun.",i:"sB"},
  C:{t:"Şarj oluyor",h:"Enerji aracınıza aktarılıyor.",i:"sC"},
  D:{t:"Şarj oluyor",h:"Havalandırmalı şarj modu.",i:"sC"},
  E:{t:"Bağlantı hatası",h:"Araçla iletişim kurulamadı. Kabloyu çıkarıp yeniden takın.",i:"sE"},
  F:{t:"İstasyon hatası",h:"Koruma devrede. Sorun sürerse yetkiliye başvurun.",i:"sE"}
};
const $=id=>document.getElementById(id);
function setText(id,v){const e=$(id);if(e&&e.textContent!==v)e.textContent=v;}
function nf(v,d){return (Number(v)||0).toLocaleString("tr-TR",{minimumFractionDigits:d,maximumFractionDigits:d});}
function fmtDur(s){s=Math.max(0,Math.floor(Number(s)||0));const h=Math.floor(s/3600),m=Math.floor(s%3600/60);
  return h?h+" sa "+String(m).padStart(2,"0")+" dk":(m?m+" dk "+String(s%60).padStart(2,"0")+" sn":s+" sn");}
let fails=0,lastState="",timer=0,busy=false;

function render(d){
  const st=ST[d.state]?d.state:"A",m=ST[st];
  const charging=st==="C"||st==="D";
  if(st!==lastState){
    lastState=st;
    $("stateCard").dataset.s=st;
    $("stateIcon").setAttribute("href","#"+m.i);
    setText("stateTitle",m.t);setText("stateHint",m.h);
    $("aracSvg").setAttribute("aria-label","Araç görseli: "+m.t.toLocaleLowerCase("tr-TR"));
  }
  setText("stationName",d.stationName||"Şarj istasyonu");
  setText("stationAddr",d.stationAddr||"-");
  // planli sarj (puant): yasakli aralikta kisa bilgi satiri
  const sb=!!Number(d.schedBlocked),sl=$("schedLine");
  sl.hidden=!sb;
  if(sb)setText("schedLine",d.schedUntil?"Puant saatleri: şarj "+d.schedUntil+" saatinde otomatik başlayacak.":"Puant saatleri: şarj bu aralıkta bekletiliyor.");
  // alarm
  const lv=Number(d.alarmLv)||0,ab=$("alarmBand");
  ab.hidden=lv<1;
  if(lv>0){ab.className="alarm l"+(lv>1?2:1);setText("alarmTxt",d.alarmTxt||"Uyarı");}
  // canli kart
  const ph=Math.min(3,Math.max(1,Number(d.phase)||1));
  const lim=Math.max(6,Number(d.limitTargetA??d.limitA)||32);
  const live=!!d.sLive;
  const kwh=live&&d.sLiveKWh!==undefined?d.sLiveKWh:d.eKWh;
  const sec=live&&d.sLiveSec!==undefined?d.sLiveSec:d.tSec;
  $("liveCard").hidden=!charging;
  if(charging){
    setText("pwr",nf((Number(d.pW)||0)/1000,1));
    setText("ekwh",nf(kwh,2));
    setText("tsec",fmtDur(sec));
    setText("phaseInfo",ph+" faz");
    [d.ia,d.ib,d.ic].forEach((a,k)=>{
      $("ph"+(k+1)).hidden=k>=ph;
      a=Number(a)||0;
      setText("i"+(k+1),nf(a,1)+" A");
      $("b"+(k+1)).style.width=Math.min(100,a/lim*100).toFixed(0)+"%";
    });
    setText("limitA",nf(lim,0));
  }
  // QR metni duruma gore
  setText("qrText",charging?"Şarjı durdurmak veya takip etmek için RTevCharge uygulamasını kullanın.":"Şarjı başlatmak için RTevCharge uygulamasından istasyondaki QR kodu okutun.");
  // seans ozeti
  const has=live||(Number(sec)||0)>0;
  setText("sessTitle",live?"Bu seans":"Son seans");
  setText("sTime",has?fmtDur(sec):"—");
  setText("sKwh",has?nf(kwh,2)+" kWh":"—");
  setText("sCost","—"); // tarife bilgisi henuz yok
  setText("ts","Son güncelleme: "+new Date().toLocaleTimeString("tr-TR"));
}

function setNet(ok){
  const s=$("sync"),n=$("netStrip");
  s.className="canli"+(ok?" on":"");
  setText("sync",ok?"CANLI":"BAĞLANTI YOK");
  if(ok){n.hidden=true;return;}
  n.hidden=false;
  n.className=fails<3?"uyari":"kotu";
  setText("netStrip",fails<3?"Bağlantı zayıf, yeniden bağlanıyor…":"İstasyona bağlantı koptu. Yeniden deneniyor…");
}

function pull(){
  clearTimeout(timer);
  if(busy)return;busy=true;
  const ac=new AbortController(),to=setTimeout(()=>ac.abort(),2500);
  fetch("/status_public",{cache:"no-store",signal:ac.signal})
    .then(r=>{if(!r.ok)throw new Error(r.status);return r.json();})
    .then(d=>{fails=0;setNet(true);render(d);})
    .catch(()=>{fails++;setNet(false);})
    .finally(()=>{clearTimeout(to);busy=false;clearTimeout(timer);if(!document.hidden)timer=setTimeout(pull,POLL_MS);});
}
document.addEventListener("visibilitychange",()=>{if(!document.hidden)pull();});

// PWA: kurulum + service worker (mevcut davranis korunur)
let deferredPrompt=null;
window.addEventListener("beforeinstallprompt",e=>{e.preventDefault();deferredPrompt=e;$("installBtn").hidden=false;});
$("installBtn").addEventListener("click",()=>{
  if(!deferredPrompt)return;
  deferredPrompt.prompt();
  deferredPrompt.userChoice.finally(()=>{deferredPrompt=null;$("installBtn").hidden=true;});
});
if("serviceWorker" in navigator){window.addEventListener("load",()=>navigator.serviceWorker.register("/sw.js").catch(()=>{}));}
// Uygulama baglantisi henuz yok (TODO: universal link / app link)
$("appBtn").addEventListener("click",e=>{if($("appBtn").getAttribute("href")==="#")e.preventDefault();});
pull();
</script>
</body></html>
)HTML";

static const char MANIFEST_JSON[] PROGMEM = R"JSON(
{
  "name": "RTevCharge",
  "short_name": "RTevCharge",
  "start_url": "/",
  "scope": "/",
  "display": "standalone",
  "background_color": "#07131f",
  "theme_color": "#0c425e",
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
const CACHE_NAME = "evse-pwa-v10";
const ASSETS = ["/", "/manifest.json", "/app-icon.svg", "/vehicle-top-art.svg", "/togg-arac-v2.webp"];

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
  // Yalnizca kullanici ekrani ve statik dosyalar onbellege alinir. Yonetim
  // sayfalari, yetkili API yanitlari ve POST istekleri hic onbellege girmez.
  if (event.request.method !== "GET" || url.origin !== self.location.origin || ASSETS.indexOf(path) < 0) {
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
<!DOCTYPE html><html lang="tr"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="color-scheme" content="light dark">
<meta name="theme-color" content="#0c425e">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="RTevCharge Yönetim"><meta name="format-detection" content="telephone=no">
<title>RTevCharge · Yönetim</title>
<link rel="icon" type="image/svg+xml" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'><rect width='64' height='64' rx='12' fill='%230c425e'/><path d='M14 44V20h14a8 8 0 0 1 0 16h-6l10 8' fill='none' stroke='%23fff' stroke-width='6' stroke-linecap='round' stroke-linejoin='round'/><circle cx='46' cy='22' r='6' fill='%23ff5a1f'/><circle cx='46' cy='42' r='6' fill='%2300b2ff'/></svg>">
<style>
:root{--lacivert:#0c425e;--turuncu:#ff5a1f;--mavi:#00b2ff;--zemin:#f3f6f8;--yuzey:#fff;--kenar:#d5dee5;--metin:#14212b;--soluk:#4d5f6d;--iyi:#11703f;--iyi-z:#e2f5ea;--kotu:#b3261e;--kotu-z:#fbe6e4;--uyari:#8a5a00;--uyari-z:#fff3d6;--dugme:#0c425e;--dugme-metin:#fff;--odak:#ff5a1f;--kod:#f0f3f6;--vurgu:#0c425e;--tab:64px;--st:env(safe-area-inset-top);--sb:env(safe-area-inset-bottom);--sl:env(safe-area-inset-left);--sr:env(safe-area-inset-right)}
@media(prefers-color-scheme:dark){:root{--zemin:#0b1a24;--yuzey:#11283a;--kenar:#24435a;--metin:#e7eef3;--soluk:#a9bccb;--iyi:#5fd896;--iyi-z:#123526;--kotu:#ff8a80;--kotu-z:#3a1a1a;--uyari:#ffc760;--uyari-z:#3a2c0e;--dugme:#00b2ff;--dugme-metin:#06141d;--kod:#0a1720;--vurgu:#00b2ff}}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html{-webkit-text-size-adjust:100%;font-size:16px}
@supports(font:-apple-system-body){html{font:-apple-system-body}}
body{margin:0;background:var(--zemin);color:var(--metin);font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;font-size:.94rem;line-height:1.5;overscroll-behavior-y:none}
[hidden]{display:none!important}
a{color:inherit}
:focus-visible{outline:3px solid var(--odak);outline-offset:2px}
button,a.nav,.seg button{touch-action:manipulation;user-select:none;-webkit-user-select:none}
button{font:inherit;font-weight:600;cursor:pointer;min-height:44px;border-radius:8px;padding:8px 16px;border:1px solid transparent;background:var(--dugme);color:var(--dugme-metin)}
button.ik{background:transparent;color:var(--metin);border-color:var(--kenar)}
button.th{background:transparent;color:var(--kotu);border-color:var(--kotu)}
button.thd{background:var(--kotu);color:#fff}
@media(prefers-color-scheme:dark){button.thd{color:#1a0505}}
button.iy{background:var(--iyi);color:#fff}
button:disabled{opacity:.5;cursor:not-allowed}
input,select{font:inherit;font-size:max(16px,1rem);width:100%;min-height:44px;padding:9px 11px;border:1px solid var(--kenar);border-radius:8px;background:var(--zemin);color:var(--metin)}
input[type=checkbox]{width:22px;min-height:22px;height:22px;accent-color:var(--vurgu);vertical-align:middle;margin:0 8px 0 0}
label{display:block;font-weight:600;font-size:.85rem;margin:0 0 4px;color:var(--soluk)}
label.ck{display:flex;align-items:center;min-height:44px;color:var(--metin);font-weight:500}
.schR{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:8px 0}.schR input[type=time]{width:auto;min-width:110px}.schR .gun{display:flex;flex-wrap:wrap;gap:8px}.schR .gun label{display:flex;gap:3px;align-items:center;font-size:13px}
.mono,.v{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;overflow-wrap:anywhere}
.lg{width:34px;height:34px;flex:none}
.marka{display:flex;align-items:center;gap:10px;font-weight:700;letter-spacing:.04em}
.marka small{display:block;font-weight:400;letter-spacing:0;opacity:.85;font-size:.75rem}
/* giris */
.giris-kap{min-height:100vh;min-height:100dvh;display:grid;place-items:center;padding:calc(16px + var(--st)) calc(16px + var(--sr)) calc(16px + var(--sb)) calc(16px + var(--sl))}
.giris{width:100%;max-width:380px;background:var(--yuzey);border:1px solid var(--kenar);border-top:4px solid var(--turuncu);border-radius:14px;padding:24px}
.giris h1{font-size:1.3rem;margin:16px 0 12px}
.giris .alan{margin-top:12px}
.pw{display:flex;gap:6px}.pw input{flex:1}.pw button{flex:none;min-width:72px}
.giris button[type=submit]{width:100%;margin-top:16px}
.mesaj{margin-top:12px;padding:10px 12px;border-radius:8px;font-size:.88rem}
.mesaj.kotu{background:var(--kotu-z);color:var(--kotu)}.mesaj.iyi{background:var(--iyi-z);color:var(--iyi)}.mesaj.uyari{background:var(--uyari-z);color:var(--uyari)}
/* ust bar */
header.ust{position:sticky;top:0;z-index:15;background:var(--lacivert);color:#fff;border-bottom:3px solid var(--turuncu);padding:var(--st) var(--sr) 0 var(--sl)}
.ust-ic{display:flex;align-items:center;gap:10px;padding:8px 12px;min-height:56px}
.ust .marka span{display:none}
.ist{flex:1;min-width:0;line-height:1.25;user-select:text;-webkit-user-select:text}
.ist b{display:block;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font-size:1rem}
.ist small{font:.78rem ui-monospace,Consolas,monospace;opacity:.85}
.ust button{background:transparent;color:#fff;border:1px solid rgba(255,255,255,.55);padding:6px 12px}
.ust button.ic{width:44px;padding:0;font-size:1.3rem}
.spin{animation:sp .8s linear infinite}@keyframes sp{to{transform:rotate(360deg)}}
#netStrip{padding:8px calc(12px + var(--sr)) 8px calc(12px + var(--sl));font-size:.85rem;font-weight:600;text-align:center}
#netStrip.kotu{background:var(--kotu-z);color:var(--kotu)}#netStrip.iyi{background:var(--iyi-z);color:var(--iyi)}#netStrip.uyari{background:var(--uyari-z);color:var(--uyari)}
/* gezinme: mobilde alt sekme, genis ekranda yan menu */
#navCard{position:fixed;left:0;right:0;bottom:0;z-index:14;display:flex;background:var(--yuzey);border-top:1px solid var(--kenar);padding:0 var(--sr) var(--sb) var(--sl)}
a.nav{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;min-height:var(--tab);text-decoration:none;color:var(--soluk);font-size:.7rem;font-weight:600;border-top:3px solid transparent}
a.nav svg{width:24px;height:24px}
a.nav.active{color:var(--vurgu);border-top-color:var(--turuncu)}
a.nav.yan{display:none}
main{padding:16px calc(12px + var(--sr)) calc(var(--tab) + var(--sb) + 24px) calc(12px + var(--sl));max-width:1240px}
@media(min-width:900px){
.ust .marka span{display:block}
.ust-ic{padding:8px 20px}
.app{display:grid;grid-template-columns:220px 1fr}
#navCard{position:sticky;top:calc(59px + var(--st));align-self:start;height:calc(100vh - 59px - var(--st));flex-direction:column;border-top:0;border-right:1px solid var(--kenar);padding:12px 8px;gap:2px}
a.nav{flex:none;flex-direction:row;justify-content:flex-start;gap:12px;padding:0 14px;min-height:46px;font-size:.92rem;border-top:0;border-left:3px solid transparent;border-radius:0 8px 8px 0}
a.nav.active{border-left-color:var(--turuncu);background:var(--kod)}
a.nav.yan{display:flex}
main{padding:20px 24px 40px}}
@media(hover:hover){a.nav:hover{background:var(--kod)}button:hover{filter:brightness(1.08)}}
/* icerik */
.sec{display:none}.sec.on{display:block}
.bas{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin:0 0 14px}
.bas h1{font-size:1.35rem;margin:0;flex:1}
.izgara{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(min(100%,300px),1fr))}
.genis{grid-column:1/-1}
.kart{background:var(--yuzey);border:1px solid var(--kenar);border-radius:12px;padding:16px;min-width:0}
.kart h2{margin:0 0 10px;font-size:.78rem;text-transform:uppercase;letter-spacing:.06em;color:var(--soluk);display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.kart h2 .s{margin-left:auto}
.rozet{display:inline-block;padding:2px 10px;border-radius:999px;font-size:.78rem;font-weight:700;background:var(--kod);color:var(--soluk);letter-spacing:0;text-transform:none}
.rozet.iyi{background:var(--iyi-z);color:var(--iyi)}.rozet.kotu{background:var(--kotu-z);color:var(--kotu)}.rozet.uyari{background:var(--uyari-z);color:var(--uyari)}
.hero{display:flex;flex-wrap:wrap;gap:14px;align-items:center}
.st{display:flex;align-items:center;gap:14px;flex:1;min-width:230px}
.stL{width:60px;height:60px;border-radius:12px;display:grid;place-items:center;font-size:2rem;font-weight:800;background:var(--kod);color:var(--soluk);flex:none}
.stL.C,.stL.D{background:var(--iyi-z);color:var(--iyi)}.stL.B{background:var(--kod);color:var(--vurgu);box-shadow:inset 0 0 0 2px var(--mavi)}.stL.E,.stL.F{background:var(--kotu-z);color:var(--kotu)}
.stT{font-size:1.15rem;font-weight:700}.stS{color:var(--soluk);font-size:.82rem}
.seg{display:flex;width:100%;max-width:420px;background:var(--kod);border:1px solid var(--kenar);border-radius:10px;padding:3px;gap:3px}
.seg button{flex:1;background:transparent;color:var(--soluk);padding:6px 8px;font-size:.85rem}
.seg button.on{background:var(--dugme);color:var(--dugme-metin)}.seg button.on.dur{background:var(--kotu);color:#fff}
.alarm{margin-top:12px;padding:10px 12px;border-radius:8px;font-weight:600;border-left:4px solid}
.alarm.l1{background:var(--uyari-z);color:var(--uyari)}.alarm.l2{background:var(--kotu-z);color:var(--kotu)}
.kpi .big{font-size:2.6rem;font-weight:800;line-height:1.05;font-variant-numeric:tabular-nums;margin:2px 0;overflow-wrap:anywhere}
.kpi .big u{text-decoration:none;font-size:1rem;color:var(--soluk);font-weight:600;margin-left:4px}
.kpi .alt{font-size:.8rem;color:var(--soluk)}
.mt{height:8px;border-radius:9px;background:var(--kod);overflow:hidden;margin-top:10px}
.mt i{display:block;height:100%;width:0;background:var(--mavi);transition:width .6s}
.ph{display:grid;grid-template-columns:30px 1fr auto;align-items:center;gap:10px;margin:12px 0}
.ph b{font-size:.8rem;color:var(--soluk)}.ph .mt{margin:0;height:14px}
.ph .v{font-size:1.25rem;font-weight:800;min-width:76px;text-align:right}
dl.bilgi{display:grid;grid-template-columns:auto 1fr;gap:8px 12px;margin:0;font-size:.9rem}
dl.bilgi dt{color:var(--soluk)}dl.bilgi dd{margin:0;text-align:right;font-family:ui-monospace,Consolas,monospace;overflow-wrap:anywhere}
.f{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,140px),1fr));gap:10px;margin-bottom:12px}
.u{position:relative}.u em{position:absolute;right:10px;top:50%;transform:translateY(-50%);font-style:normal;font-size:.8rem;color:var(--soluk);pointer-events:none}.u input{padding-right:36px}
.not{font-size:.85rem;color:var(--soluk);margin:0 0 12px}
.satir{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.kaydet{position:sticky;bottom:calc(var(--tab) + var(--sb) + 8px);z-index:5;margin-top:14px;display:flex;align-items:center;gap:8px;flex-wrap:wrap;padding:10px 12px;border-radius:12px;background:var(--lacivert);color:#fff;border-bottom:3px solid var(--turuncu)}
.kaydet span{flex:1;font-size:.85rem;opacity:.85}.kaydet span.kirli{opacity:1;font-weight:700;color:#ffb38f}
.kaydet button.ik{color:#fff;border-color:rgba(255,255,255,.55)}
@media(min-width:900px){.kaydet{bottom:12px}}
.wl{display:flex;flex-direction:column;gap:6px}
.wi{display:flex;align-items:center;gap:10px;padding:8px 10px;border:1px solid var(--kenar);border-radius:8px;background:var(--zemin)}
.wi .n{flex:1;font-weight:600;overflow-wrap:anywhere}.wi .s{font-size:.75rem;color:var(--soluk)}
.sig{display:inline-flex;align-items:flex-end;gap:2px}.sig i{width:4px;background:var(--kenar);border-radius:1px}.sig i.on{background:var(--iyi)}
.ver{display:grid;grid-template-columns:1fr auto 1fr;gap:10px;align-items:center;text-align:center;margin-bottom:14px}
.ver>div{padding:12px 8px;border-radius:10px;background:var(--kod);border:1px solid var(--kenar)}
.ver small{display:block;font-size:.7rem;font-weight:700;color:var(--soluk);letter-spacing:.06em}
.ver b{display:block;font:800 1.4rem ui-monospace,Consolas,monospace;margin-top:2px}
.ver .ok{border:0;background:none;color:var(--soluk);font-size:1.3rem;padding:0}
.ver .yeni{border-color:var(--turuncu);box-shadow:inset 0 0 0 1px var(--turuncu)}
.tehlike{border:2px solid var(--kotu)}.tehlike h2{color:var(--kotu)}
.dz{display:flex;gap:12px;align-items:center;justify-content:space-between;flex-wrap:wrap;padding:12px 0;border-bottom:1px solid var(--kenar)}
.dz:last-child{border-bottom:0;padding-bottom:0}.dz p{margin:2px 0 0;font-size:.85rem;color:var(--soluk);max-width:520px}
.tk{overflow-x:auto}
table{width:100%;border-collapse:collapse;font-size:.85rem}
th,td{padding:8px 10px;text-align:left;border-bottom:1px solid var(--kenar);white-space:nowrap}
th{font-size:.72rem;text-transform:uppercase;letter-spacing:.05em;color:var(--soluk)}
tr:last-child td{border-bottom:0}
.ov{position:fixed;inset:0;z-index:30;background:rgba(6,20,29,.6);display:flex;align-items:flex-end;justify-content:center;padding:16px calc(16px + var(--sr)) calc(16px + var(--sb)) calc(16px + var(--sl))}
@media(min-width:600px){.ov{align-items:center}}
.dlg{background:var(--yuzey);border-radius:14px;max-width:440px;width:100%;padding:20px;border-top:4px solid var(--turuncu)}
.dlg.red{border-top-color:var(--kotu)}.dlg h3{margin:0 0 6px;font-size:1.1rem}.dlg p{margin:0 0 12px;color:var(--soluk);font-size:.9rem}
.dlg .satir{justify-content:flex-end;margin-top:14px}
#toast{position:fixed;left:50%;bottom:calc(var(--tab) + var(--sb) + 14px);transform:translateX(-50%);z-index:40;background:var(--lacivert);color:#fff;padding:10px 16px;border-radius:10px;font-size:.9rem;font-weight:600;opacity:0;pointer-events:none;transition:opacity .25s;max-width:calc(100vw - 32px)}
#toast.on{opacity:1}
@media(prefers-reduced-motion:reduce){*{transition:none!important;animation:none!important}}
</style></head><body>
<svg width="0" height="0" style="position:absolute" aria-hidden="true"><defs>
<symbol id="lg" viewBox="0 0 64 64"><rect width="64" height="64" rx="12" fill="#0c425e"/><g transform="translate(5 4.4) scale(.6)"><path d="M69.93 88.81C69.64 88.50 69.21 87.87 68.97 87.41C68.03 85.58 67.66 84.92 67.34 84.45C67.15 84.18 67.00 83.89 67.00 83.81C67.00 83.72 66.80 83.36 66.55 83.00C66.30 82.64 66.04 82.17 65.97 81.96C65.89 81.74 65.71 81.39 65.55 81.19C65.40 80.98 65.15 80.53 65.00 80.19C64.85 79.84 64.63 79.45 64.50 79.31C64.37 79.17 64.15 78.79 64.01 78.46C63.87 78.13 63.62 77.68 63.46 77.46C63.30 77.24 63.09 76.87 63.00 76.62C62.90 76.38 62.70 76.02 62.55 75.81C62.39 75.61 62.15 75.16 62.00 74.81C61.85 74.47 61.63 74.08 61.50 73.94C61.37 73.80 61.15 73.41 61.00 73.06C60.85 72.72 60.61 72.27 60.45 72.06C60.30 71.86 60.10 71.49 60.00 71.25C59.91 71.01 59.70 70.63 59.54 70.41C59.38 70.19 59.13 69.74 58.99 69.41C58.85 69.08 58.63 68.70 58.50 68.56C58.37 68.42 58.15 68.04 58.01 67.71C57.87 67.38 57.62 66.93 57.46 66.71C57.29 66.49 57.10 66.14 57.03 65.92C56.96 65.70 56.70 65.23 56.45 64.87C56.20 64.52 56.00 64.15 56.00 64.06C56.00 63.97 55.81 63.63 55.59 63.30C55.36 62.96 55.10 62.49 55.00 62.25C54.90 62.01 54.64 61.54 54.41 61.20C54.19 60.87 54.00 60.53 54.00 60.45C54.00 60.38 53.77 59.98 53.50 59.58C53.22 59.17 53.00 58.77 53.00 58.68C53.00 58.60 52.81 58.25 52.59 57.92C52.36 57.59 52.10 57.12 52.00 56.88C51.90 56.63 51.64 56.16 51.41 55.83C51.19 55.50 51.00 55.15 51.00 55.07C51.00 54.98 50.85 54.69 50.66 54.42C50.48 54.16 50.15 53.60 49.93 53.19C49.71 52.77 49.32 52.04 49.07 51.56C48.81 51.08 48.53 50.60 48.44 50.50C48.34 50.40 48.15 50.04 48.01 49.70C47.86 49.37 47.61 48.93 47.45 48.74C47.28 48.54 47.08 48.17 46.99 47.91C46.91 47.65 46.71 47.27 46.55 47.06C46.40 46.86 46.15 46.41 46.01 46.07C45.86 45.74 45.58 45.27 45.37 45.04C44.55 44.10 44.97 43.41 46.60 43.07C47.13 42.96 48.01 42.70 48.56 42.50C49.11 42.30 49.95 42.04 50.41 41.94C50.88 41.84 51.53 41.64 51.85 41.50C52.17 41.37 52.94 41.14 53.55 41.00C54.17 40.86 54.93 40.63 55.24 40.49C55.55 40.35 56.29 40.13 56.87 40.00C57.46 39.87 58.27 39.62 58.69 39.46C59.10 39.29 59.77 39.09 60.19 39.01C60.60 38.93 61.27 38.73 61.69 38.56C62.10 38.39 62.92 38.15 63.50 38.01C64.78 37.71 66.88 36.95 67.02 36.72C67.61 35.81 67.87 33.73 67.87 29.88C67.87 26.38 67.81 25.82 67.20 23.52C66.62 21.32 64.81 19.46 62.75 18.94C62.30 18.83 61.57 18.62 61.13 18.49C59.59 18.02 46.90 17.57 29.94 17.37C1.55 17.05 2.44 17.08 2.32 16.45C2.24 16.08 2.13 11.92 2.11 8.90C2.09 6.72 2.12 6.33 2.29 6.16C2.44 6.00 3.00 5.92 4.65 5.79C5.84 5.70 7.57 5.54 8.50 5.44C9.43 5.34 12.47 5.14 15.25 5.00C18.04 4.86 21.52 4.63 23.00 4.50C24.48 4.37 27.71 4.17 30.19 4.07C36.51 3.80 39.37 3.64 41.75 3.44C49.21 2.80 62.24 2.63 66.19 3.12C68.66 3.43 73.36 4.83 74.22 5.51C74.40 5.64 74.79 5.87 75.10 6.00C75.66 6.25 75.88 6.41 77.20 7.54C78.19 8.37 79.77 10.17 80.10 10.82C80.24 11.10 80.44 11.40 80.55 11.49C80.65 11.58 80.85 11.95 80.99 12.32C81.13 12.70 81.39 13.21 81.55 13.47C81.72 13.73 81.92 14.20 81.99 14.51C82.06 14.83 82.28 15.50 82.49 16.01C82.69 16.52 82.94 17.47 83.05 18.12C83.16 18.78 83.35 19.75 83.49 20.28C83.93 21.99 84.06 24.12 84.05 29.69C84.05 35.34 83.96 36.71 83.44 38.50C83.27 39.08 83.07 39.92 83.00 40.37C82.92 40.81 82.73 41.43 82.57 41.74C82.41 42.06 82.16 42.69 82.01 43.15C81.86 43.61 81.63 44.12 81.51 44.27C81.39 44.43 81.16 44.84 81.01 45.19C80.86 45.53 80.64 45.92 80.51 46.06C80.39 46.20 80.17 46.53 80.02 46.80C79.53 47.65 76.55 50.88 76.24 50.88C76.19 50.88 75.83 51.13 75.43 51.44C75.03 51.75 74.64 52.00 74.55 52.00C74.47 52.00 74.13 52.19 73.80 52.42C73.46 52.65 72.96 52.91 72.69 52.99C72.41 53.08 71.93 53.31 71.62 53.50C71.32 53.70 70.81 53.92 70.50 54.00C68.94 54.41 68.50 55.39 69.44 56.38C69.60 56.55 69.85 56.96 69.99 57.29C70.13 57.62 70.38 58.07 70.54 58.29C70.71 58.51 70.90 58.86 70.97 59.08C71.04 59.30 71.30 59.77 71.55 60.13C71.80 60.48 72.00 60.85 72.00 60.93C72.00 61.02 72.15 61.31 72.34 61.58C72.52 61.84 72.85 62.40 73.07 62.81C74.16 64.87 74.36 65.22 74.67 65.67C74.85 65.94 75.00 66.23 75.00 66.32C75.00 66.40 75.19 66.75 75.41 67.08C75.64 67.41 75.90 67.88 76.00 68.12C76.09 68.37 76.30 68.73 76.45 68.94C76.61 69.14 76.85 69.59 77.00 69.94C77.15 70.28 77.37 70.67 77.50 70.81C77.63 70.95 77.85 71.34 78.00 71.69C78.15 72.03 78.35 72.40 78.45 72.51C78.61 72.69 78.89 73.20 79.94 75.25C80.14 75.65 80.41 76.08 80.52 76.21C80.64 76.33 80.85 76.71 80.99 77.04C81.13 77.37 81.38 77.82 81.54 78.04C81.70 78.26 81.91 78.63 82.00 78.88C82.10 79.12 82.36 79.59 82.59 79.92C82.81 80.25 83.00 80.60 83.00 80.68C83.00 80.77 83.15 81.06 83.34 81.33C83.67 81.80 84.05 82.48 84.91 84.12C85.14 84.57 85.48 85.16 85.67 85.42C85.85 85.69 86.00 85.98 86.00 86.07C86.00 86.15 86.20 86.53 86.45 86.89C87.37 88.26 87.57 88.89 87.16 89.10C87.04 89.17 83.23 89.25 78.70 89.30L70.46 89.38L69.93 88.81Z" fill="#fff" fill-rule="evenodd"/><rect x="18" y="43.4" width="15.4" height="29" fill="#00b2ff"/></g></symbol>
<symbol id="iL" viewBox="0 0 24 24"><path d="M13 2 4 14h7l-1 8 9-12h-7z" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></symbol>
<symbol id="iS" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="4" width="18" height="16" rx="2"/><path d="M7 9h10M7 13h6M7 17h8"/></g></symbol>
<symbol id="iC" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M4 6h10M4 12h4M12 12h8M4 18h12"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/></g></symbol>
<symbol id="iW" viewBox="0 0 24 24"><g fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M2 9a15 15 0 0 1 20 0M5 13a10 10 0 0 1 14 0M8.5 16.5a5 5 0 0 1 7 0"/><circle cx="12" cy="20" r="1"/></g></symbol>
<symbol id="iT" viewBox="0 0 24 24"><path d="M9 3v6l-5 9a2 2 0 0 0 2 3h12a2 2 0 0 0 2-3l-5-9V3M8 3h8" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></symbol>
<symbol id="iO" viewBox="0 0 24 24"><path d="M12 3v12M7 10l5 5 5-5M4 19h16" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></symbol>
</defs></svg>

<!-- GIRIS: jeton yoksa yalnizca bu gorunur -->
<main class="giris-kap" id="loginView">
<form class="giris" id="loginForm" autocomplete="on">
  <div class="marka"><svg class="lg"><use href="#lg"/></svg><span>RTevCharge<small>Elektrikli araç şarjı · yönetim paneli</small></span></div>
  <h1>Yönetici girişi</h1>
  <div class="alan"><label for="lgUser">Kullanıcı adı</label><input id="lgUser" name="username" value="admin" readonly autocomplete="username"></div>
  <div class="alan"><label for="lgPass">Parola</label><div class="pw"><input id="lgPass" name="password" type="password" required autocomplete="current-password" maxlength="64" autocapitalize="off" spellcheck="false"><button type="button" class="ik" onclick="togglePw('lgPass',this)">Göster</button></div></div>
  <div class="mesaj" id="lgMsg" role="alert" hidden></div>
  <button type="submit" id="lgBtn">Giriş</button>
  <p class="not" style="margin:12px 0 0" id="lgInfo">5 hatalı denemeden sonra giriş geçici olarak kilitlenir.</p>
</form>
</main>

<!-- PANEL: girisle acilir -->
<div id="appView" hidden>
<header class="ust"><div class="ust-ic">
  <div class="marka"><svg class="lg"><use href="#lg"/></svg><span>RTevCharge<small>Elektrikli araç şarjı · yönetim paneli</small></span></div>
  <div class="ist" title="Dokun: IP'yi kopyala" onclick="copyIp()"><b id="stationTitle">-</b><small><span id="ipTop">-</span> · <span id="hostTop">-</span></small></div>
  <button class="ic" id="refreshBtn" onclick="refreshAll()" aria-label="Yenile">&#8635;</button>
  <button onclick="logout()">Çıkış</button>
</div></header>
<div id="netStrip" role="status" aria-live="polite" hidden></div>
<div id="pwWarn" class="mesaj uyari" role="alert" hidden style="margin:0;border-radius:0;text-align:center"><b>Varsayılan parola kullanılıyor.</b> Güvenlik için <a href="#settings">Ayarlar › Panel parolası</a> bölümünden hemen değiştirin.</div>

<div class="app">
<nav id="navCard" aria-label="Ana menü">
  <a class="nav" id="navAdmin" href="#live" data-s="liveCard"><svg><use href="#iL"/></svg>Canlı</a>
  <a class="nav" id="navStatus" href="#status" data-s="statusCard"><svg><use href="#iS"/></svg>Durum</a>
  <a class="nav" id="navCalibration" href="#settings" data-s="calibrationCard"><svg><use href="#iC"/></svg>Ayarlar</a>
  <a class="nav" id="navWifi" href="#wifi" data-s="wifiCard"><svg><use href="#iW"/></svg>Wi-Fi</a>
  <a class="nav yan" id="navTest" href="#test" data-s="testCard"><svg><use href="#iT"/></svg>Test</a>
  <a class="nav" id="navOta" href="#ota" data-s="otaCard"><svg><use href="#iO"/></svg>Yazılım</a>
</nav>
<main id="icerik">

<!-- CANLI -->
<section class="sec" id="liveCard">
<div class="izgara">
  <div class="kart genis">
    <div class="hero">
      <div class="st"><div class="stL" id="stBox">-</div><div><div class="stT" id="stTxt">-</div><div class="stS"><span id="badgeState">STATE:-</span> · Ham <span id="sRawTop">-</span> · Mod <span id="modeTxt">-</span> · <span id="badgeRelay">R: -</span></div></div></div>
      <div class="seg" id="modeSeg"><button data-m="0" onclick="chargeCmd(0)">Otomatik</button><button data-m="1" onclick="chargeCmd(1)">Başlat</button><button data-m="2" class="dur" onclick="chargeCmd(2)">Durdur</button></div>
    </div>
    <div class="alarm" id="alarmBox" hidden></div>
  </div>
  <div class="kart kpi"><h2>Anlık güç</h2><div class="big"><span id="pwr">-</span><u>kW</u></div><div class="alt">Tavan ≈ <span id="pMax">-</span> kW</div><div class="mt"><i id="pBar"></i></div></div>
  <div class="kart kpi"><h2>Enerji</h2><div class="big"><span id="ekwh">-</span><u>kWh</u></div><div class="alt">Canlı seans <span id="sLiveKWh">-</span> kWh</div></div>
  <div class="kart kpi"><h2>Süre</h2><div class="big"><span id="tsec">-</span></div><div class="alt">Faz <span id="phase">-</span> · Limit <span id="limitTxt">-</span> A</div></div>
  <div class="kart">
    <h2>Faz akımları <span class="s rozet" id="iAvgTag">ORT -</span></h2>
    <div class="ph"><b>L1</b><div class="mt"><i id="b1"></i></div><span class="v"><span id="i1">-</span> A</span></div>
    <div class="ph"><b>L2</b><div class="mt"><i id="b2"></i></div><span class="v"><span id="i2">-</span> A</span></div>
    <div class="ph"><b>L3</b><div class="mt"><i id="b3"></i></div><span class="v"><span id="i3">-</span> A</span></div>
  </div>
  <div class="kart">
    <h2>Control Pilot <button class="ik" type="button" onclick="fetchLogs()">Log</button></h2>
    <dl class="bilgi"><dt>State kararlı / ham</dt><dd><span id="sStb">-</span> / <span id="sRaw">-</span></dd><dt>CP yüksek / düşük</dt><dd><span id="cH">-</span> / <span id="cL">-</span></dd><dt>ADC yüksek / düşük</dt><dd><span id="aH">-</span> / <span id="aL">-</span></dd><dt>PP GPIO16</dt><dd><span id="ppV">-</span> / <span id="ppRaw">-</span></dd><dt>Röle</dt><dd id="rLbl">-</dd></dl>
    <pre id="logBox" hidden style="white-space:pre-wrap;font-size:.75rem;max-height:180px;overflow:auto"></pre>
  </div>
</div>
</section>

<!-- DURUM -->
<section class="sec" id="statusCard">
<div class="izgara">
  <div class="kart" id="displayPanel"><h2>Ağ ve cihaz <span class="s rozet" id="connTag">-</span></h2>
    <dl class="bilgi"><dt>Wi-Fi</dt><dd><span id="wifiSsid">-</span> (<span id="wifiLoc">-</span>)</dd><dt>IP</dt><dd id="ip">-</dd><dt>Host</dt><dd id="host">-</dd><dt>MAC</dt><dd id="mac">-</dd><dt>İstasyon kodu</dt><dd id="stationCode">-</dd><dt>Adres</dt><dd id="stationAddr">-</dd><dt>Son güncelleme</dt><dd id="lastUpd">-</dd></dl></div>
  <div class="kart"><h2>Sıfırlama geçmişi</h2>
    <dl class="bilgi"><dt>Toplam</dt><dd id="rstTotal">0</dd><dt>Anlık / geçmiş</dt><dd><span id="rstNow">0</span> / <span id="rstHist">0</span></dd><dt>Son işlem</dt><dd><span id="rstLastMode">YOK</span> @ <span id="rstLastSec">0</span>s</dd><dt>Firmware</dt><dd id="fwSmall">-</dd></dl></div>
  <div class="kart genis"><h2>Son şarj seansları <button class="s ik" onclick="loadHistory()">Yenile</button></h2>
    <div class="tk"><table><thead><tr><th>#</th><th>Başlangıç</th><th>Süre</th><th>Enerji</th><th>Ort. güç</th><th>Faz</th></tr></thead><tbody id="histBody"><tr><td colspan="6">-</td></tr></tbody></table></div></div>
</div>
</section>

<!-- AYARLAR (kalibrasyon + istasyon + guvenlik) -->
<section class="sec" id="calibrationCard">
<div class="izgara" id="calForm">
  <div class="kart"><h2>Zamanlama ve limit</h2><p class="not">Bu kartta kontaktör gecikmesiz seviye sürülür (GPIO4, C/D iken HIGH). Röle açma/bırakma süreleri uygulanmaz.</p><div class="f" data-f="lInt|Döngü|ms;stb|Stable count;onD|Röle açma (uygulanmaz)|ms;offD|Röle bırakma (uygulanmaz)|ms;limitASet|Akım limiti 6-32|A"></div></div>
  <div class="kart"><h2>CP kalibrasyon</h2><p class="not">State A ve PWM kapalıyken bölücü 12.00 V'a otomatik çekilir. Eşikler kalıcı kaydedilir, otomatik değişmez.</p><div class="f" data-f="div|Divider;thb|TH_B|V;thc|TH_C|V;thd|TH_D|V;the|TH_E|V"></div></div>
  <div class="kart genis"><h2>Akım kalibrasyonu</h2>
    <p class="not">Varsayılan: 0-10 A için Ical 12.0; 10-30 A için ek offset 1.0 A. Tavsiye: rngLowMax 10, rngLowOff 0, rngMidMax 30, rngMidOff 1.</p>
    <div class="f" data-f="icalA|Ical L1;icalB|Ical L2;icalC|Ical L3;ioffA|Offset L1|A;ioffB|Offset L2|A;ioffC|Offset L3|A;rngLowMax|0-10A maks.|A;rngLowOff|0-10A offset|A;rngMidMax|10-30A maks.|A;rngMidOff|10-30A offset|A"></div></div>
  <div class="kart genis"><h2>Pens ampermetre yardımı <span class="s rozet">Canlı <span id="liveIa">-</span> / <span id="liveIb">-</span> / <span id="liveIc">-</span> · ort <span id="liveIAvg">-</span> A</span></h2>
    <p class="not">1) 0-10 A'de pens değerlerini girip "Ical doldur". 2) 10-30 A'de girip "Offset doldur". 3) Alttan Uygula.</p>
    <div class="f" data-f="clampLowA|Pens düşük L1;clampLowB|Pens düşük L2;clampLowC|Pens düşük L3"><div><label>&nbsp;</label><button class="ik" style="width:100%" onclick="fillClampLow()">Ical doldur</button></div></div>
    <div class="f" data-f="clampMidA|Pens orta L1;clampMidB|Pens orta L2;clampMidC|Pens orta L3"><div><label>&nbsp;</label><button class="ik" style="width:100%" onclick="fillClampMid()">Offset doldur</button></div></div></div>
  <div class="kart genis"><h2>İstasyon</h2><div class="f">
    <div><label for="stationNameSet">İstasyon adı</label><input id="stationNameSet" placeholder="Boşsa MAC'ten otomatik"></div>
    <div><label for="mapLat">Enlem</label><input id="mapLat" inputmode="decimal" placeholder="37.94559"></div>
    <div><label for="mapLng">Boylam</label><input id="mapLng" inputmode="decimal" placeholder="32.58082"></div></div>
    <dl class="bilgi"><dt>Üretilen adres</dt><dd id="stationAddr2">-</dd></dl></div>
</div>
<div class="kaydet"><span id="calDirty">Değişiklik yok</span><button class="ik" onclick="pull(true)">Geri al</button><button onclick="applyCal()">Uygula</button></div>

<div class="izgara" style="margin-top:14px">
  <div class="kart"><h2>Panel parolası</h2>
    <form id="pwForm" autocomplete="off">
      <input type="text" name="username" value="admin" autocomplete="username" hidden>
      <div class="f" style="grid-template-columns:1fr">
        <div><label for="pwCur">Mevcut parola</label><input id="pwCur" type="password" required autocomplete="current-password"></div>
        <div><label for="pwNew">Yeni parola (en az 10 karakter)</label><input id="pwNew" type="password" required minlength="10" maxlength="64" autocomplete="new-password"></div>
        <div><label for="pwNew2">Yeni parola (tekrar)</label><input id="pwNew2" type="password" required minlength="10" maxlength="64" autocomplete="new-password"></div>
      </div>
      <div class="satir"><button type="submit">Parolayı değiştir</button><button type="button" class="ik" onclick="togglePw('pwNew',this);togglePw('pwNew2');togglePw('pwCur')">Göster</button></div>
      <div class="mesaj" id="pwMsg" role="status" hidden></div>
    </form></div>
  <div class="kart"><h2>Oturum</h2>
    <dl class="bilgi"><dt>Kullanıcı</dt><dd>admin</dd><dt>Otomatik çıkış</dt><dd><span id="idleLeft">15:00</span> sonra</dd></dl>
    <div class="satir" style="margin-top:12px"><button class="ik" onclick="location.hash='test'">Servis testleri</button><button class="th" onclick="logout()">Çıkış yap</button></div></div>
  <div class="kart genis" id="cloudBox"><h2>Bulut bağlantısı <span class="s rozet" id="cloudTag">-</span></h2>
    <div class="mesaj uyari" id="cloudWarn" style="margin:0 0 12px" hidden>Bulut ayarı yapılmadı: istasyon kodunu kontrol edip sunucudan alınan gizli anahtarı girin.</div>
    <dl class="bilgi"><dt>Durum</dt><dd id="cloudPhase">-</dd><dt>Son yoklama</dt><dd id="cloudAge">-</dd><dt>Son HTTP</dt><dd id="cloudHttp">-</dd><dt>Son komut</dt><dd id="cloudCmd">-</dd><dt>Gizli anahtar</dt><dd id="cloudSecretState">-</dd></dl>
    <label class="ck"><input type="checkbox" id="cloudOn">Bulut bağlantısı açık</label>
    <div class="f">
      <div><label for="cloudCode">İstasyon kodu</label><input id="cloudCode" autocapitalize="characters" spellcheck="false" maxlength="16" placeholder="Boşsa MAC'ten"></div>
      <div><label for="cloudUrl">Sunucu adresi</label><input id="cloudUrl" autocapitalize="off" spellcheck="false" maxlength="95" placeholder="https://sarj.rotosis.com"></div>
      <div><label for="cloudSecret">Gizli anahtar</label><input id="cloudSecret" type="password" autocomplete="off" autocapitalize="off" spellcheck="false" maxlength="43" placeholder="Değişmeyecekse boş"></div>
    </div>
    <div class="satir"><button onclick="saveCloud()">Kaydet</button><button class="th" onclick="clearCloudSecret()">Anahtarı sil</button></div>
    <label class="ck" style="margin-top:8px"><input type="checkbox" id="cloudReq" onclick="return toggleCloudReq(event)">Şarjı yalnızca uygulamadan başlat</label>
    <p class="not">Açıkken araç takılsa da şarj, uygulamadan "Başlat" gelmeden başlamaz. Başladıktan sonra bağlantı kopsa da şarj sürer; uygulamadan durdurma, aracı ayırma ya da paneldeki Durdur ile biter.</p></div>
  <div class="kart genis" id="schedBox"><h2>Planlı şarj (puant) <span class="s rozet" id="schedTag">-</span></h2>
    <p class="not" id="schedNow">-</p>
    <label class="ck"><input type="checkbox" id="schedOn">Yasaklı saatlerde şarj etme</label>
    <div id="schedRows"></div>
    <div class="satir"><button class="ik" onclick="schedAdd()">Aralık ekle</button><button class="ik" onclick="schedPeak()">Puant (17–22)</button><button onclick="schedSave()">Kaydet</button></div>
    <p class="not" style="margin:10px 0 0">Türkiye saati (UTC+3). Gece yarısını aşan aralık olabilir (örn. 23:00–06:00); aralık başladığı güne aittir. En çok 3 aralık. Cihaz saati bilinmiyorsa kısıtlama uygulanmaz.</p></div>
</div>
</section>

<!-- WIFI -->
<section class="sec" id="wifiCard">
<div class="izgara">
  <div class="kart"><h2>Bağlantı profili</h2>
    <div class="mesaj uyari" style="margin:0 0 12px"><div id="wifiConnectedInfo">Mevcut bağlantı: bağlı değil</div><div id="wifiCurrentMode">Bağlantı profili: varsayılan liste</div></div>
    <label class="ck"><input type="checkbox" id="wifiEnabled">Özel profil etkin</label>
    <div class="f">
      <div><label for="wifiSsidCfg">SSID</label><input id="wifiSsidCfg" autocapitalize="off" spellcheck="false"></div>
      <div><label for="wifiPassCfg">Şifre</label><input id="wifiPassCfg" type="password" placeholder="Değişmeyecekse boş" autocomplete="new-password"></div>
      <div><label for="wifiModeCfg">IP modu</label><select id="wifiModeCfg" onchange="toggleWifiModeFields()"><option value="dhcp">DHCP</option><option value="static">Statik</option></select></div>
    </div>
    <div id="wifiStaticFields" class="f" data-f="wifiIpCfg|IP||192.168.1.200;wifiGwCfg|Gateway||192.168.1.1;wifiSubnetCfg|Subnet||255.255.255.0;wifiDns1Cfg|DNS1||8.8.8.8;wifiDns2Cfg|DNS2||1.1.1.1"></div>
    <button onclick="applyWifiSettings()">Kaydet ve bağlan</button></div>
  <div class="kart"><h2>Çevredeki ağlar <button class="s ik" onclick="scanWifiList()">Tara</button></h2>
    <p class="not" id="wifiScanStatus">Tarama hazır değil</p><div id="wifiScanList" class="wl"></div></div>
</div>
</section>

<!-- TEST -->
<section class="sec" id="testCard">
<div class="bas"><h1>Servis testleri</h1><button class="ik" onclick="location.hash='settings'">Ayarlar'a dön</button></div>
<div class="mesaj uyari" style="margin:0 0 14px"><b>Servis modu.</b> Bu komutlar otomatik akışa anında müdahale eder; araç bağlıyken dikkatli kullanın.</div>
<div class="izgara">
  <div class="kart"><h2>Röle <span class="s rozet" id="relayTag">-</span></h2>
    <div class="satir"><button class="iy" onclick="send('/relay?on=1')">Manuel aç</button><button class="th" onclick="send('/relay?on=0')">Manuel bırak</button><button class="ik" onclick="send('/relay_auto?en=1')">Otomatiğe dön</button></div></div>
  <div class="kart"><h2>MOSFET darbe testi</h2>
    <div class="satir"><button class="th" onclick="send('/pulse_reset')">RESET (GPIO 7)</button><button class="iy" onclick="send('/pulse_set')">SET (GPIO 15)</button></div>
    <p class="not" style="margin:10px 0 0">Her basış 100 ms HIGH darbe gönderir.</p></div>
</div>
</section>

<!-- YAZILIM / OTA -->
<section class="sec" id="otaCard">
<div class="izgara">
  <div class="kart"><h2>Yazılım güncelleme <span class="s rozet" id="otaTag">-</span></h2>
    <div class="ver"><div><small>KURULU</small><b id="otaCurVer">-</b></div><div class="ok">&rarr;</div><div id="otaRemoteBox"><small>SUNUCUDA</small><b id="otaRemoteVer">-</b></div></div>
    <div class="satir"><button class="ik" onclick="runOtaCheckAdmin()">Kontrol et</button><button id="otaInstallBtn" onclick="runOtaInstall()" disabled>Güncellemeyi yükle</button><button class="ik" onclick="openUpdate()">.bin yükle</button></div>
    <p class="not" style="margin:10px 0 0">Kontrol yalnızca yeni sürümü bulur; yükleme ayrıca onay ister, şarj durur ve cihaz yeniden başlar.</p></div>
  <div class="kart"><h2>Teşhis</h2>
    <dl class="bilgi"><dt>Çalışan bölüm</dt><dd id="otaPart">-</dd><dt>Image state</dt><dd id="otaImgState">-</dd><dt>Durum</dt><dd id="otaStatus">-</dd><dt>Son kontrol</dt><dd id="otaAge">-</dd><dt>Hata</dt><dd id="otaError">Hata yok</dd></dl></div>
  <div class="kart genis tehlike"><h2>Tehlikeli bölge</h2>
    <div class="dz"><div><b>Önceki OTA sürümüne dön</b><p>Aktif olmayan OTA bölümünü seçip yeniden başlatır.</p></div><button class="th" onclick="runBootPrev()">Önceki sürüme dön</button></div>
    <div class="dz"><div><b>Fabrika yazılımına dön</b><p>USB ile yüklenen kurtarma (factory) sürümünü açar.</p></div><button class="thd" onclick="runBootFactory()">Fabrikaya dön</button></div>
    <div class="dz"><div><b>Şarj verilerini sıfırla</b><p>Seçilen veriler kalıcı olarak silinir.</p>
      <div class="satir"><label class="ck"><input type="checkbox" id="rstOptNow" checked>Anlık sayaç</label><label class="ck"><input type="checkbox" id="rstOptHist" checked>Seans geçmişi</label></div></div>
      <button class="thd" onclick="runDataReset()">Verileri sıfırla</button></div>
  </div>
</div>
</section>
</main></div></div>

<div class="ov" id="ov" hidden><div class="dlg" id="dlg" role="dialog" aria-modal="true" aria-labelledby="dlgT"><h3 id="dlgT"></h3><p id="dlgM"></p>
<div id="dlgW"><label for="dlgI">Onay için <b id="dlgK"></b> yazın</label><input id="dlgI" autocomplete="off" autocapitalize="characters"></div>
<div class="satir"><button class="ik" onclick="ask.close()">Vazgeç</button><button id="dlgOk">Onayla</button></div></div></div>
<div id="toast" role="status" aria-live="polite"></div>

<script>
/* Yetkili uc noktalar: /status /history /charge_cmd /calib_apply /wifi_scan /wifi_apply
   /ota_check /ota_install /boot_prev /boot_factory /relay /relay_auto /pulse_reset /pulse_set
   /data_reset /api/logout /api/password /api/upload_ticket /api/cloud_cfg /api/schedule -> hepsi "Authorization: Bearer <jeton>" ister, yoksa 401.
   Acik: /api/login (POST JSON {user,password} -> {token,ttl,mustChange}; hatada 401 {left}, kilitte 429 {retryAfter}). */
const PAGE_MODE="__PAGE_MODE__",IDLE_MS=15*60e3,MAX_TRY=5,LOCK_S=60;
let TOK=null,paused=false,t,last=null,calDirty=false,wifiDirty=false,fails=0,lastAct=Date.now(),idleWarned=false,tries=0,lockUntil=0,pollT=null,idleT=null;
const $=id=>document.getElementById(id);
/* Form alanlari data-f tanimindan uretilir: "id|Etiket|birim|ornek;..." (id'ler aynen korunur) */
document.querySelectorAll('[data-f]').forEach(b=>{b.innerHTML=b.dataset.f.split(';').map(x=>{const[id,l,u,ph]=x.split('|'),i='<input id="'+id+'" inputmode="decimal"'+(ph?' placeholder="'+ph+'"':'')+'>';return'<div><label for="'+id+'">'+l+'</label>'+(u?'<div class="u">'+i+'<em>'+u+'</em></div>':i)+'</div>';}).join('')+b.innerHTML;});
Object.assign($('limitASet'),{type:'number',min:6,max:32,step:1});
function T(id,v){const e=$(id);if(e)e.textContent=v;}
function num(v,f=0){const n=Number(v);return Number.isFinite(n)?n:f;}
function toast(m){const e=$('toast');e.textContent=m;e.classList.add('on');clearTimeout(toast.t);toast.t=setTimeout(()=>e.classList.remove('on'),2800);}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function p(){paused=true;clearTimeout(t);}
function r(){clearTimeout(t);t=setTimeout(()=>{paused=false;},3000);}
document.addEventListener('focusin',e=>{if(/INPUT|SELECT/.test(e.target.tagName))p();});
document.addEventListener('focusout',e=>{if(/INPUT|SELECT/.test(e.target.tagName))r();});
function togglePw(id,b){const e=$(id);e.type=e.type==='password'?'text':'password';if(b)b.textContent=e.type==='password'?'Göster':'Gizle';}

/* --- Jeton deposu: sessionStorage, olmazsa bellek (WebView uyumu) --- */
const SS={get(k){try{return sessionStorage.getItem(k);}catch(e){return null;}},set(k,v){try{v==null?sessionStorage.removeItem(k):sessionStorage.setItem(k,v);}catch(e){}}};


/* --- Yetkili istek katmani --- */
function api(u,o={}){
  o.cache='no-store';o.headers=Object.assign({},o.headers,{Authorization:'Bearer '+TOK});
  return fetch(u,o).then(x=>{if(x.status===401){logout('Oturum sona erdi, tekrar giriş yapın.',true);throw new Error('Yetkisiz');}net(true);return x;},e=>{net(false);throw e;});
}
function getJSON(u){return api(u).then(x=>{if(!x.ok)throw new Error('HTTP '+x.status);return x.json();});}
function call(u,o){return api(u,o).then(x=>x.text().then(s=>{if(!x.ok)throw new Error(s||('HTTP '+x.status));return s;}));}
function send(u){call(u).then(()=>{toast('Komut gönderildi');pull(true);}).catch(e=>toast(e.message||'İstek başarısız'));}
function fetchLogs(){call('/logs').then(t=>{const e=$('logBox');if(e){e.hidden=false;e.textContent=t&&t.length?t:'(bos)';}}).catch(e=>toast(e.message||'Log alinamadi'));}

/* --- Baglanti seridi --- */
function strip(cls,msg){const s=$('netStrip');s.className=cls;s.textContent=msg;s.hidden=!msg;}
function net(ok){
  if(ok){if(fails){strip('iyi','Bağlantı yeniden kuruldu');setTimeout(()=>{if(!fails)strip('', '');},2500);}fails=0;}
  else{fails++;strip('kotu','Cihaza ulaşılamıyor · yeniden bağlanıyor… ('+fails+')');}
}
addEventListener('offline',()=>strip('kotu','Telefonun ağ bağlantısı yok'));
addEventListener('online',()=>{strip('uyari','Ağ geri geldi, yenileniyor…');pull(true);});

/* --- Giris / cikis --- */
function lgMsg(cls,m){const e=$('lgMsg');e.className='mesaj '+cls;e.textContent=m;e.hidden=!m;}
function lockTick(){const s=Math.ceil((lockUntil-Date.now())/1000),b=$('lgBtn');
  if(s>0){b.disabled=true;b.textContent='Kilitli · '+s+' sn';lgMsg('kotu','Çok fazla hatalı deneme. '+s+' sn sonra tekrar deneyin.');setTimeout(lockTick,1000);}
  else{b.disabled=false;b.textContent='Giriş';tries=0;lgMsg('','');SS.set('evseLock',null);}}
function lock(sec){lockUntil=Date.now()+sec*1000;SS.set('evseLock',String(lockUntil));lockTick();}
function loginFail(left){tries++;const k=left!=null?left:MAX_TRY-tries;if(k<=0){lock(LOCK_S);return;}
  lgMsg(k<=2?'kotu':'uyari','Parola hatalı. Kalan deneme: '+k+(k<=2?' — sonra '+LOCK_S+' sn kilitlenir.':''));$('lgPass').select();}
$('loginForm').addEventListener('submit',e=>{
  e.preventDefault();if(lockUntil>Date.now())return;
  const pw=$('lgPass').value,b=$('lgBtn');b.disabled=true;b.textContent='Kontrol ediliyor…';
  const done=()=>{if(lockUntil<=Date.now()){b.disabled=false;b.textContent='Giriş';}};
  fetch('/api/login',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json'},body:JSON.stringify({user:'admin',password:pw})})
  .then(x=>x.json().catch(()=>({})).then(j=>{done();
    if(x.ok&&j.token)startSession(j.token,j.mustChange);
    else if(x.status===429)lock(num(j.retryAfter,LOCK_S));
    else if(x.status===401)loginFail(j.left);
    else lgMsg('kotu','Giriş yapılamadı (HTTP '+x.status+').');}))
  .catch(()=>{done();lgMsg('kotu','Cihaza ulaşılamadı. Ağ bağlantısını kontrol edip tekrar deneyin.');});
});
function pwBand(on){$('pwWarn').hidden=!on;}
function startSession(tok,mustChange){
  TOK=tok;SS.set('evseTok',tok);tries=0;$('lgPass').value='';lgMsg('','');
  $('loginView').hidden=true;$('appView').hidden=false;lastAct=Date.now();idleWarned=false;
  pwBand(!!mustChange);
  applyPageMode();pull(true);
  clearInterval(pollT);pollT=setInterval(()=>{if(!document.hidden)pull(false);},3000);
  clearInterval(idleT);idleT=setInterval(idleCheck,1000);
}
function logout(msg,expired){
  if(TOK&&!expired)fetch('/api/logout',{method:'POST',headers:{Authorization:'Bearer '+TOK}}).catch(()=>{});
  TOK=null;SS.set('evseTok',null);clearInterval(pollT);clearInterval(idleT);ask.close();
  ['wifiPassCfg','pwCur','pwNew','pwNew2','lgPass'].forEach(i=>$(i).value='');
  $('appView').hidden=true;$('loginView').hidden=false;strip('','');fails=0;
  lgMsg(msg?'uyari':'',msg||'');setTimeout(()=>$('lgPass').focus(),50);
}
['pointerdown','keydown','touchstart'].forEach(ev=>addEventListener(ev,()=>{lastAct=Date.now();idleWarned=false;},{passive:true}));
function idleCheck(){if(!TOK)return;const left=IDLE_MS-(Date.now()-lastAct);
  if(left<=0){logout('15 dakika işlem yapılmadığı için çıkış yapıldı.');return;}
  if(left<60e3&&!idleWarned){idleWarned=true;toast('1 dakika içinde otomatik çıkış yapılacak');}
  T('idleLeft',fmtTime(left/1000));}
document.addEventListener('visibilitychange',()=>{if(!document.hidden&&TOK){idleCheck();if(TOK)pull(true);}});

/* --- Onay diyalogu --- */
const up=s=>s.trim().toLocaleUpperCase('tr').replace(/İ/g,'I');
const ask={open(o){T('dlgT',o.t);T('dlgM',o.m);$('dlg').className='dlg'+(o.red?' red':'');$('dlgW').hidden=!o.w;T('dlgK',o.w||'');$('dlgI').value='';
  const ok=$('dlgOk');ok.className=o.red?'thd':'';ok.textContent=o.ok||'Onayla';
  ok.onclick=()=>{if(o.w&&up($('dlgI').value)!==up(o.w)){toast('Onay kelimesi hatalı');return;}ask.close();o.go();};
  $('ov').hidden=false;if(o.w)setTimeout(()=>$('dlgI').focus(),50);},close(){$('ov').hidden=true;}};
$('ov').addEventListener('click',e=>{if(e.target.id==='ov')ask.close();});

/* --- Gezinme --- */
const SECS=['liveCard','statusCard','calibrationCard','wifiCard','testCard','otaCard'];
const HASH={live:'liveCard',admin:'liveCard',status:'statusCard',settings:'calibrationCard',calibration:'calibrationCard',wifi:'wifiCard',test:'testCard',ota:'otaCard'};
function showPage(id){SECS.forEach(s=>$(s).classList.toggle('on',s===id));
  document.querySelectorAll('a.nav').forEach(a=>a.classList.toggle('active',a.dataset.s===id||(id==='testCard'&&a.id==='navCalibration')));
  if(id==='statusCard')loadHistory();scrollTo(0,0);}
function applyPageMode(){const m=PAGE_MODE.indexOf('__')===0?'admin':PAGE_MODE;showPage(HASH[location.hash.slice(1)]||HASH[m]||'liveCard');}
addEventListener('hashchange',()=>{if(TOK)applyPageMode();});
function refreshAll(){const b=$('refreshBtn');b.classList.add('spin');pull(true);if($('statusCard').classList.contains('on'))loadHistory();setTimeout(()=>b.classList.remove('spin'),800);}
function copyIp(){const ip=$('ipTop').textContent;if(navigator.clipboard&&ip!=='-')navigator.clipboard.writeText(ip).then(()=>toast('IP kopyalandı: '+ip),()=>{});}

/* --- Bicimleyiciler --- */
const ST={A:'Araç bağlı değil',B:'Araç bağlı, hazır',C:'Şarj oluyor',D:'Şarj (havalandırma)',E:'Hata: CP kısa devre',F:'Hata: EVSE arızası'};
function fmtTime(s){s=Math.max(0,Math.floor(s));const h=Math.floor(s/3600),z=n=>String(n).padStart(2,'0');return(h?h+':':'')+z(Math.floor(s%3600/60))+':'+z(s%60);}
function fmtOtaAge(ms){const s=Math.round(num(ms)/1000);return!s?'hemen şimdi':s<60?s+' sn önce':Math.floor(s/60)+' dk '+(s%60)+' sn önce';}
function fmtDate(s){if(s<1e9)return s+' s';const d=new Date(s*1000);return d.toLocaleString('tr-TR',{day:'2-digit',month:'2-digit',hour:'2-digit',minute:'2-digit'});}
function compareVersions(a,b){const pa=String(a||'').split('.').map(x=>parseInt(x,10)||0),pb=String(b||'').split('.').map(x=>parseInt(x,10)||0);for(let i=0;i<Math.max(pa.length,pb.length,4);i++){const x=pa[i]||0,y=pb[i]||0;if(x!==y)return x>y?1:-1;}return 0;}
function setInput(id,v,force){const e=$(id);if(!e||v===undefined)return;if(document.activeElement===e&&!force)return;if(e.type==='checkbox')e.checked=!!v;else e.value=v;}

/* --- Canli veri --- */
const CAL=[['lInt','lInt'],['onD','onD'],['offD','offD'],['stb','stable'],['div','div'],['thb','thb'],['thc','thc'],['thd','thd'],['the','the'],['icalA','icalA'],['icalB','icalB'],['icalC','icalC'],['ioffA','ioffA'],['ioffB','ioffB'],['ioffC','ioffC'],['rngLowMax','rngLowMax'],['rngMidMax','rngMidMax'],['rngLowOff','rngLowOff'],['rngMidOff','rngMidOff'],['mapLat','mapLat'],['mapLng','mapLng']];
function render(d,force){
  last=d;const st=d.state||'-',lim=num(d.limitA,16)||16,ph=num(d.phase,1),kw=num(d.pW)/1000,kwMax=lim*230*ph/1000;
  T('stationTitle',d.stationName||'RTevCharge');T('ipTop',d.ip||'-');T('hostTop',d.host||'-');T('lastUpd',new Date().toLocaleTimeString('tr-TR'));const ct=$('connTag');ct.className='s rozet '+(d.staOk?'iyi':'kotu');ct.textContent=d.staOk?'Wi-Fi bağlı':'Wi-Fi yok';
  const sb=$('stBox');sb.textContent=st;sb.className='stL '+st;T('stTxt',ST[st]||'Bilinmiyor');
  T('badgeState','STATE:'+st);T('sRawTop',d.stateRaw||'-');T('modeTxt',d.mode||'-');T('badgeRelay','R: '+(d.rLbl||'-'));
  document.querySelectorAll('#modeSeg button').forEach(b=>b.classList.toggle('on',+b.dataset.m===num(d.modeId)));
  const al=$('alarmBox');al.hidden=!(d.alarmLv>0);al.className='alarm l'+d.alarmLv;al.textContent=d.alarmTxt||'';
  T('sStb',st);T('sRaw',d.stateRaw);T('cH',num(d.cpHigh).toFixed(2)+'V');T('cL',num(d.cpLow).toFixed(2)+'V');T('aH',num(d.adcHigh).toFixed(3)+'V');T('aL',num(d.adcLow).toFixed(3)+'V');T('ppV',num(d.ppVolt).toFixed(3)+'V');T('ppRaw',String(num(d.ppRaw,0)));
  [['i1',d.ia,'b1'],['i2',d.ib,'b2'],['i3',d.ic,'b3']].forEach(a=>{const v=num(a[1]),b=$(a[2]);T(a[0],v.toFixed(2));b.style.width=Math.min(100,v/lim*100)+'%';b.style.background=v>lim+1?'var(--kotu)':'';});
  T('pwr',kw.toFixed(2));T('pMax',kwMax.toFixed(1));$('pBar').style.width=Math.min(100,kw/kwMax*100)+'%';
  T('ekwh',num(d.eKWh).toFixed(3));T('sLiveKWh',d.sLive?num(d.sLiveKWh).toFixed(3):'-');T('tsec',fmtTime(num(d.tSec)));T('phase',ph+'F');T('limitTxt',lim.toFixed(0));T('iAvgTag','ORT '+num(d.iAvg).toFixed(2)+' A');
  T('rLbl',d.rLbl||'-');const rt=$('relayTag');rt.textContent=d.rLbl||'-';rt.className='s rozet '+(d.rLbl==='SET'?'iyi':'');
  T('wifiSsid',d.wifiSsid||'-');T('wifiLoc',d.wifiLoc||'-');T('ip',d.ip||'-');T('host',d.host||'-');T('mac',d.mac||'-');T('stationCode',d.stationCode||'-');T('stationAddr',d.stationAddr||'-');T('stationAddr2',d.stationAddr||'-');
  T('rstTotal',d.rstTotal||0);T('rstNow',d.rstNow||0);T('rstHist',d.rstHist||0);T('rstLastMode',d.rstLastMode||'YOK');T('rstLastSec',d.rstLastSec||0);
  T('liveIa',num(d.ia).toFixed(2));T('liveIb',num(d.ib).toFixed(2));T('liveIc',num(d.ic).toFixed(2));T('liveIAvg',num(d.iAvg).toFixed(2));
  renderOta(d);if(d.pwMustChange!==undefined)pwBand(!!d.pwMustChange);renderCloud(d.cloud);renderSched(d.sched);
  if(!calDirty||force){CAL.forEach(a=>setInput(a[0],d[a[1]],force));setInput('limitASet',d.limitTargetA!==undefined?d.limitTargetA:d.limitA,force);setInput('stationNameSet',d.stationName||'',force);}
  if(!wifiDirty||force)applyWifiStatus(d);
  if(force){calDirty=false;wifiDirty=false;markCal();}
}
function pull(force=false){if(!TOK||(paused&&!force))return;getJSON('/status').then(d=>{if(TOK)render(d,force);}).catch(()=>{});}

/* --- OTA ve tehlikeli islemler --- */
function renderOta(d){T('otaCurVer',d.otaCur||'-');T('otaRemoteVer',d.otaRemote||'-');T('fwSmall',d.otaCur||'-');T('otaPart',d.otaPart||'-');T('otaImgState',d.otaImgState||'-');T('otaStatus',d.otaStatus||'-');T('otaAge',fmtOtaAge(d.otaAgeMs));T('otaError',d.otaErr?d.otaErr:'Hata yok');updateOtaInstallButton(d);}
function updateOtaInstallButton(d){const has=compareVersions(d.otaRemote,d.otaCur)>0,er=!!d.otaErr,tg=$('otaTag'),b=$('otaInstallBtn');
  b.disabled=!has;b.textContent=has?'Güncellemeyi yükle':'Güncel';$('otaRemoteBox').className=has?'yeni':'';
  tg.className='s rozet '+(er?'kotu':has?'uyari':'iyi');tg.textContent=er?'Hata':has?'Yeni sürüm var':'Güncel';}
function runOtaCheckAdmin(){call('/ota_check').then(()=>{T('otaStatus','check_requested');toast('Sunucu kontrol ediliyor');setTimeout(()=>pull(true),4000);}).catch(()=>toast('OTA kontrolü gönderilemedi'));}
function runOtaInstall(){const cur=$('otaCurVer').textContent,rem=$('otaRemoteVer').textContent;
  if(compareVersions(rem,cur)<=0){toast('Yüklenecek yeni sürüm yok');return;}
  ask.open({t:'Güncelleme yüklensin mi?',m:cur+' → '+rem+'. Şarj durur, cihaz yeniden başlar; işlem bitene kadar gücü kesmeyin.',ok:'Yükle ve yeniden başlat',go:()=>call('/ota_install').then(()=>{T('otaStatus','install_requested');T('otaError','Admin onayı verildi, yükleme başlıyor');toast('Yükleme başladı');}).catch(()=>toast('OTA yükleme isteği gönderilemedi'))});}
/* .bin yukleme sayfasi: Bearer ile 60 sn'lik tek kullanimlik bilet alinir, /update?t=... acilir */
function openUpdate(){api('/api/upload_ticket',{method:'POST'}).then(x=>{if(!x.ok)throw new Error('HTTP '+x.status);return x.json();}).then(j=>{location='/update?t='+encodeURIComponent(j.t);}).catch(()=>toast('Yükleme sayfası açılamadı'));}
function runBootPrev(){ask.open({t:'Önceki sürüme dönülsün mü?',m:'Aktif olmayan OTA bölümü seçilip cihaz yeniden başlatılacak.',red:1,ok:'Geri dön',go:()=>call('/boot_prev').then(s=>toast(s||'Yeniden başlıyor')).catch(e=>toast(e.message))});}
function runBootFactory(){ask.open({t:'Fabrika yazılımına dön',m:'Cihaz kurtarma sürümüyle açılacak; sonra OTA ile yeniden güncellemeniz gerekir.',w:'FABRİKA',red:1,ok:'Fabrikaya dön',go:()=>call('/boot_factory').then(s=>toast(s||'Yeniden başlıyor')).catch(e=>toast(e.message))});}
function runDataReset(){const n=$('rstOptNow').checked,h=$('rstOptHist').checked;if(!n&&!h){toast('En az bir seçenek işaretleyin');return;}
  ask.open({t:'Veriler silinsin mi?',m:'Silinecek: '+[n&&'anlık sayaç',h&&'seans geçmişi'].filter(Boolean).join(' + ')+'. Geri alınamaz.',w:'SIFIRLA',red:1,ok:'Sil',go:()=>call('/data_reset?now='+(+n)+'&hist='+(+h)).then(()=>{toast('Veriler sıfırlandı');pull(true);loadHistory();}).catch(()=>toast('Sıfırlama başarısız'))});}
function chargeCmd(m){const go=()=>call('/charge_cmd?m='+m).then(()=>{toast(['Otomatik moda alındı','Şarj başlatıldı','Şarj durduruldu'][m]);pull(true);}).catch(()=>toast('Komut gönderilemedi'));
  if(m===2)ask.open({t:'Şarj durdurulsun mu?',m:'Röle ve PWM hemen kapatılır.',red:1,ok:'Durdur',go});else go();}
function loadHistory(){getJSON('/history').then(h=>{const it=(h.items||[]).slice().reverse();$('histBody').innerHTML=it.length?it.map((x,i)=>'<tr><td>'+(i+1)+'</td><td>'+fmtDate(num(x.s))+'</td><td>'+fmtTime(num(x.d))+'</td><td>'+num(x.e).toFixed(2)+' kWh</td><td>'+(num(x.p)/1000).toFixed(1)+' kW</td><td>'+num(x.ph)+'F</td></tr>').join(''):'<tr><td colspan="6">Kayıt yok</td></tr>';}).catch(()=>{});}

/* --- Ayarlar --- */
function markCal(){const e=$('calDirty');e.textContent=calDirty?'Kaydedilmemiş değişiklik var':'Değişiklik yok';e.className=calDirty?'kirli':'';}
$('calForm').addEventListener('input',e=>{if(!/^clamp/.test(e.target.id)){calDirty=true;markCal();}});
function fillClamp(k){let c=0;['A','B','C'].forEach(ph=>{const ref=num($('clamp'+k+ph).value),live=num($('liveI'+ph).textContent);
  if(k==='Low'&&ref>0&&live>.2){$('ical'+ph).value=(num($('ical'+ph).value,12)*ref/live).toFixed(2);c++;}
  if(k==='Mid'&&ref>0&&live>0){$('ioff'+ph).value=(ref-live).toFixed(2);c++;}});
  if(!c){toast('En az bir pens değeri girin ve canlı akım olsun');return;}calDirty=true;markCal();toast(k==='Low'?'Ical alanları dolduruldu':'Offset alanları dolduruldu');}
function fillClampLow(){fillClamp('Low');}
function fillClampMid(){fillClamp('Mid');}
function applyCal(){const q=new URLSearchParams(),F={lInt:'lInt',onD:'onD',offD:'offD',s:'stb',limitA:'limitASet',div:'div',thb:'thb',thc:'thc',thd:'thd',the:'the',icalA:'icalA',icalB:'icalB',icalC:'icalC',ioffA:'ioffA',ioffB:'ioffB',ioffC:'ioffC',rngLowMax:'rngLowMax',rngMidMax:'rngMidMax',rngLowOff:'rngLowOff',rngMidOff:'rngMidOff',stationName:'stationNameSet',mapLat:'mapLat',mapLng:'mapLng'};
  for(const k in F)q.append(k,$(F[k]).value);const lim=num($('limitASet').value);if(lim<6||lim>32){toast('Akım limiti 6-32 A olmalı');return;}
  if(document.activeElement)document.activeElement.blur();
  call('/calib_apply?'+q).then(()=>{toast('Ayarlar uygulandı');calDirty=false;markCal();paused=false;pull(true);}).catch(()=>toast('Ayarlar kaydedilemedi'));}
$('pwForm').addEventListener('submit',e=>{e.preventDefault();const m=$('pwMsg'),say=(c,s)=>{m.className='mesaj '+c;m.textContent=s;m.hidden=false;},c=$('pwCur').value,n=$('pwNew').value;
  if(n.length<10)return say('kotu','Yeni parola en az 10 karakter olmalı.');
  if(n!==$('pwNew2').value)return say('kotu','Yeni parolalar eşleşmiyor.');
  if(n===c)return say('kotu','Yeni parola mevcut paroladan farklı olmalı.');
  api('/api/password',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({current:c,new:n})})
  .then(x=>x.json().catch(()=>({})).then(j=>{if(!x.ok)return say('kotu',j.error||'Parola değiştirilemedi.');
    if(j.token){TOK=j.token;SS.set('evseTok',j.token);}pwBand(false);['pwCur','pwNew','pwNew2'].forEach(i=>$(i).value='');say('iyi','Parola değiştirildi. Diğer oturumlar kapatıldı.');}))
  .catch(()=>say('kotu','Cihaza ulaşılamadı.'));});

/* --- Bulut baglantisi (gizli anahtar asla geri okunmaz; yalnizca ayarli/ayarsiz) --- */
let cloudDirty=false,schedDirty=false,SCH=null;
['cloudCode','cloudUrl','cloudSecret','cloudOn'].forEach(i=>$(i).addEventListener('input',()=>{cloudDirty=true;}));
const CPH={bagli:'Bağlı',hata:'Hata (yeniden deneniyor)',anahtar_reddedildi:'Anahtar reddedildi',wifi_yok:'Wi-Fi yok',saat_bekleniyor:'Saat bekleniyor (NTP)',ayar_yok:'Bulut ayarı yapılmadı',kapali:'Kapalı',baslatiliyor:'Başlatılıyor'};
function renderCloud(c){if(!c)return;const tg=$('cloudTag');
  tg.className='s rozet '+(c.ok?'iyi':c.enabled?'kotu':'uyari');tg.textContent=c.ok?'Bağlı':c.enabled?'Kopuk':(c.on?'Ayar yok':'Kapalı');
  $('cloudWarn').hidden=!(c.on&&!c.secretSet);
  T('cloudPhase',CPH[c.phase]||c.phase||'-');T('cloudAge',c.ageSec>=0?fmtOtaAge(c.ageSec*1000):'henüz yok');T('cloudHttp',c.lastHttp||'-');
  T('cloudCmd',c.lastCmdId?('#'+c.lastCmdId+' '+c.lastCmdType+(c.lastCmdOk?' (başarılı)':' (reddedildi)')):'-');T('cloudSecretState',c.secretSet?'Ayarlı':'Ayarsız');
  const rq=$('cloudReq');rq.checked=!!c.requireCloudStart;rq.disabled=!c.enabled&&!c.requireCloudStart;
  if(!cloudDirty){setInput('cloudOn',!!c.on,true);setInput('cloudCode',c.code||'',true);setInput('cloudUrl',c.url||'',true);}}
function cloudPost(b,msg){return api('/api/cloud_cfg',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
  .then(x=>x.json().catch(()=>({})).then(j=>{if(!x.ok)throw new Error(j.error||('HTTP '+x.status));toast(msg);renderCloud(j);return j;}));}
function saveCloud(){const s=$('cloudSecret').value.trim(),code=$('cloudCode').value.trim().toUpperCase(),url=$('cloudUrl').value.trim();
  if(s&&!/^[A-Za-z0-9_-]{43}$/.test(s)){toast('Gizli anahtar 43 karakter olmalı (A-Z a-z 0-9 _ -)');return;}
  if(code&&!/^[A-Z0-9_-]{3,16}$/.test(code)){toast('İstasyon kodu 3-16 karakter (A-Z 0-9 _ -)');return;}
  if(url&&!/^https:\/\//i.test(url)){toast('Sunucu adresi https:// ile başlamalı');return;}
  const b={on:$('cloudOn').checked,code:code,url:url};if(s)b.secret=s;
  cloudPost(b,'Bulut ayarı kaydedildi').then(()=>{$('cloudSecret').value='';cloudDirty=false;}).catch(e=>toast(e.message));}
function clearCloudSecret(){ask.open({t:'Gizli anahtar silinsin mi?',m:'Bulut bağlantısı durur ve "yalnızca uygulamadan başlat" kapanır. Yeniden bağlanmak için sunucudan alınan anahtarı tekrar girmeniz gerekir.',red:1,ok:'Sil',
  go:()=>cloudPost({clearSecret:true,requireCloudStart:false},'Anahtar silindi').catch(e=>toast(e.message))});}
function toggleCloudReq(e){e.preventDefault();const on=!(last&&last.cloud&&last.cloud.requireCloudStart);
  ask.open({t:on?'Şarj yalnızca uygulamadan başlasın mı?':'Uygulama onayı kaldırılsın mı?',
    m:on?'Araç takılınca şarj kendiliğinden başlamaz; uygulamadan "Başlat" gerekir. Bağlantı yokken yeni şarj başlatılamaz (süren şarj devam eder).':'Araç takılınca şarj mevcut moda göre kendiliğinden başlar.',
    ok:on?'Aç':'Kapat',go:()=>cloudPost({requireCloudStart:on},on?'Uygulamadan başlatma açıldı':'Uygulamadan başlatma kapatıldı').then(()=>pull(true)).catch(x=>toast(x.message))});return false;}

/* --- Planli sarj (yasakli saat araliklari) --- */
const DAYS=['Pzt','Sal','Çar','Per','Cum','Cmt','Paz'];
function schedRowsHtml(){$('schedRows').innerHTML=SCH.windows.length?SCH.windows.map((w,i)=>'<div class="schR"><input type="time" value="'+esc(w.s)+'" aria-label="Başlangıç" onchange="SCH.windows['+i+'].s=this.value;schedDirty=true"><span>–</span><input type="time" value="'+esc(w.e)+'" aria-label="Bitiş" onchange="SCH.windows['+i+'].e=this.value;schedDirty=true"><span class="gun">'+DAYS.map((d,k)=>'<label><input type="checkbox"'+((w.d>>k)&1?' checked':'')+' onchange="schedDay('+i+','+k+',this.checked)">'+d+'</label>').join('')+'</span><button class="th" onclick="schedDel('+i+')">Sil</button></div>').join(''):'<p class="not">Aralık yok</p>';}
function schedDay(i,k,on){const w=SCH.windows[i];w.d=on?(w.d|(1<<k)):(w.d&~(1<<k));schedDirty=true;}
function schedAdd(){if(SCH.windows.length>=3){toast('En çok 3 aralık');return;}SCH.windows.push({s:'17:00',e:'22:00',d:127});schedDirty=true;schedRowsHtml();}
function schedDel(i){SCH.windows.splice(i,1);schedDirty=true;schedRowsHtml();}
function schedPeak(){SCH.windows=[{s:'17:00',e:'22:00',d:127}];$('schedOn').checked=true;schedDirty=true;schedRowsHtml();toast('Puant aralığı (17:00–22:00, her gün) hazır; Kaydet ile uygulayın');}
$('schedOn').addEventListener('change',()=>{schedDirty=true;});
function fmtLeft(s){s=num(s,-1);if(s<0)return'';const h=Math.floor(s/3600),m=Math.ceil(s%3600/60);return h?h+' sa '+m+' dk':m+' dk';}
function renderSched(s){if(!s)return;const tg=$('schedTag');
  tg.className='s rozet '+(!s.on?'':!s.timeOk?'uyari':s.blocked?'kotu':'iyi');tg.textContent=!s.on?'Kapalı':!s.timeOk?'Saat yok':s.blocked?'Yasaklı':'Serbest';
  const left=s.nextChangeInSec>=0?fmtLeft(s.nextChangeInSec):'';
  T('schedNow',(!s.on?'Zamanlayıcı kapalı: şarj saat kısıtlaması olmadan çalışır.':!s.timeOk?'Cihaz saati henüz alınamadı (NTP); kısıtlama uygulanmıyor.':
    s.blocked?('Şu an yasaklı aralıkta'+(s.until?' (bitiş '+s.until+(left?', '+left+' sonra':'')+')':'')+'. Bekleyen şarj bitişte otomatik başlar.'):
    ('Şu an serbest'+(s.until?'; sonraki yasaklı aralık '+s.until+' saatinde başlar':'')+'.'))+(s.now?' Cihaz saati: '+s.now+'.':''));
  if(!schedDirty||!SCH){SCH={on:!!s.on,windows:(s.windows||[]).map(w=>({s:w.s,e:w.e,d:w.d}))};setInput('schedOn',SCH.on,true);schedRowsHtml();}}
function schedSave(){if(!SCH)return;const on=$('schedOn').checked,ws=SCH.windows;
  for(const w of ws){if(!w.s||!w.e){toast('Saatleri doldurun');return;}if(w.s===w.e){toast('Başlangıç ve bitiş aynı olamaz');return;}if(!w.d){toast('Her aralıkta en az bir gün seçin');return;}}
  if(on&&!ws.length){toast('En az bir aralık ekleyin');return;}
  const go=()=>api('/api/schedule',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({on:on,windows:ws})})
    .then(x=>x.json().catch(()=>({})).then(j=>{if(!x.ok)throw new Error(j.error||('HTTP '+x.status));schedDirty=false;SCH=null;renderSched(j);toast('Planlı şarj kaydedildi');pull(true);})).catch(e=>toast(e.message));
  if(on)ask.open({t:'Planlı şarj açılsın mı?',m:'Yasaklı saatlerde şarj güvenle durdurulur, araç bağlıysa aralık bitince kendiliğinden devam eder.',ok:'Kaydet',go});else go();}

/* --- Wi-Fi --- */
$('wifiCard').addEventListener('input',()=>{wifiDirty=true;});
function toggleWifiModeFields(){$('wifiStaticFields').hidden=$('wifiModeCfg').value!=='static';}
function applyWifiStatus(d){setInput('wifiEnabled',!!d.wifiCfgEnabled,true);setInput('wifiSsidCfg',d.wifiCfgSsid||'',true);setInput('wifiModeCfg',d.wifiCfgMode||'dhcp',true);
  ['Ip','Gw','Subnet','Dns1','Dns2'].forEach(k=>setInput('wifi'+k+'Cfg',d['wifiCfg'+k]||'',true));toggleWifiModeFields();
  T('wifiCurrentMode',d.wifiCfgEnabled?'Bağlantı profili: '+(d.wifiCfgSsid||'-')+' / '+(d.wifiCfgMode||'dhcp').toUpperCase():'Bağlantı profili: varsayılan liste');
  T('wifiConnectedInfo',d.wifiSsid&&d.wifiSsid!=='-'?'Mevcut bağlantı: '+d.wifiSsid+(d.wifiLoc&&d.wifiLoc!=='-'?' / '+d.wifiLoc:''):'Mevcut bağlantı: bağlı değil');}
function bars(r){const n=r>-55?4:r>-67?3:r>-78?2:1;let s='<span class="sig" aria-label="'+n+'/4">';for(let i=1;i<=4;i++)s+='<i class="'+(i<=n?'on':'')+'" style="height:'+(i*3+3)+'px"></i>';return s+'</span>';}
function renderWifiScanList(items){const h=$('wifiScanList');if(!items||!items.length){h.innerHTML='<p class="not">Tarama sonucu boş</p>';return;}
  items.sort((a,b)=>b.rssi-a.rssi);h.innerHTML=items.map(it=>'<div class="wi">'+bars(num(it.rssi))+'<span class="n">'+esc(it.ssid||'(gizli)')+'</span><span class="s">'+num(it.rssi)+' dBm</span><button class="ik" type="button" onclick="pickWifiSsid('+esc(JSON.stringify(String(it.ssid||'')))+')">Seç</button></div>').join('');}
function pickWifiSsid(s){$('wifiSsidCfg').value=s;$('wifiEnabled').checked=true;wifiDirty=true;$('wifiPassCfg').focus();}
function scanWifiList(){T('wifiScanStatus','Tarama yapılıyor…');getJSON('/wifi_scan').then(d=>{renderWifiScanList(d.items||[]);T('wifiScanStatus','Tarama tamamlandı · '+(d.items||[]).length+' ağ');}).catch(()=>T('wifiScanStatus','Tarama başarısız'));}
function applyWifiSettings(){const q=new URLSearchParams(),v=id=>$(id).value;
  q.append('wifiEnabled',$('wifiEnabled').checked?'1':'0');q.append('wifiSsid',v('wifiSsidCfg'));q.append('wifiPass',v('wifiPassCfg'));q.append('wifiDhcp',v('wifiModeCfg')==='static'?'0':'1');
  ['Ip','Gw','Subnet','Dns1','Dns2'].forEach(k=>q.append('wifi'+k,v('wifi'+k+'Cfg')));
  ask.open({t:'Wi-Fi ayarları kaydedilsin mi?',m:'Cihaz yeni ağa bağlanmayı deneyecek. Yanlış ayarda panele erişim kesilebilir, IP değişebilir.',ok:'Kaydet',go:()=>call('/wifi_apply?'+q).then(()=>{toast('Kaydedildi, cihaz yeniden bağlanıyor');$('wifiPassCfg').value='';wifiDirty=false;pull(true);}).catch(()=>toast('Wi-Fi ayarları kaydedilemedi'))});}

/* --- Acilis --- */
lockUntil=num(SS.get('evseLock'));if(lockUntil>Date.now())lockTick();
const saved=SS.get('evseTok');if(saved)startSession(saved);else $('lgPass').focus();
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
  noteHttpResponseSent();
  server.send_P(200, "text/html; charset=utf-8", USER_HTML);
}
// Yonetim paneli HTML kabugu korumasiz servis edilir; icinde gizli veri yoktur.
// Yetki yalnizca veri uclarinda (Bearer jeton) aranir.
// ~50 KB'lik sayfa RAM'e kopyalanmadan, yer tutucu yerinde degistirilerek akitilir.
static void sendMainPage(const char* pageMode) {
  noteWebActivity();
  static const char kModeKey[] = "__PAGE_MODE__";
  const String modeVal = pageMode ? pageMode : "admin";
  const char* tpl = MAIN_HTML;
  const size_t tplLen = strlen(tpl);

  // 1. tur: toplam uzunluk; 2. tur: gonderim.
  size_t total = 0;
  for (int pass = 0; pass < 2; ++pass) {
    const char* cur = tpl;
    if (pass == 1) {
      server.sendHeader("Cache-Control", "no-store");
      server.setContentLength(total);
      server.send(200, "text/html; charset=utf-8", "");
    }
    for (;;) {
      const char* hit = strstr(cur, kModeKey);
      const size_t keyLen = sizeof(kModeKey) - 1;
      const String* val = &modeVal;
      size_t chunk = hit ? (size_t)(hit - cur) : (size_t)(tpl + tplLen - cur);
      if (pass == 0) {
        total += chunk + (hit ? val->length() : 0);
      } else {
        if (chunk) server.sendContent(cur, chunk);
        if (hit && val->length()) server.sendContent(*val);
      }
      if (!hit) break;
      cur = hit + keyLen;
    }
  }
  noteHttpResponseSent();
}
static void handleAdmin() { sendMainPage("admin"); }
static void handleCalibrationPage() { sendMainPage("calibration"); }
static void handleWifiPage() { sendMainPage("wifi"); }
static void handlePing() { noteWebActivity(); noteHttpResponseSent(); server.send(200, "text/plain", "OK"); }
static void handleLogs() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  static char buf[2048];
  app_log_dump(buf, sizeof(buf), 40);
  noteHttpResponseSent();
  server.send(200, "text/plain; charset=utf-8", buf[0] ? buf : "(log bos)\n");
}
static void handleManifest() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/manifest+json", MANIFEST_JSON); }
static void handleServiceWorker() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "application/javascript", SERVICE_WORKER_JS); }
static void handleAppIcon() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "image/svg+xml", APP_ICON_SVG); }
static void handleVehicleTopArt() { noteWebActivity(); noteHttpResponseSent(); server.send_P(200, "image/svg+xml", VEHICLE_TOP_ART_SVG); }
// Kullanici ana ekranindaki arac fotografi. Herkese acik (auth yok);
// PROGMEM (eslenmis flash) uzerinden RAM'e kopyalanmadan gonderilir.
static void handleToggAracWebp() {
  noteWebActivity();
  server.sendHeader("Cache-Control", "public, max-age=86400");
  noteHttpResponseSent();
  server.send_P(200, "image/webp", reinterpret_cast<PGM_P>(TOGG_ARAC_V2_WEBP), TOGG_ARAC_V2_WEBP_LEN);
}
static void handleOtaCheck() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
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

// /update sayfasi: panelden alinan 60 sn'lik tek kullanimlik ?t= biletiyle acilir.
// Bilet burada yalnizca kontrol edilir; asil tuketim POST yuklemesinin basinda olur.
static void handleManualUpdatePage() {
  noteWebActivity();
  server.sendHeader("Cache-Control", "no-store");
  const String ticket = server.arg("t");
  if (!auth_check_upload_ticket(ticket, false)) {
    noteHttpResponseSent();
    server.send(401, "text/html; charset=utf-8",
                F("<!doctype html><html><head><meta charset='utf-8'>"
                  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Yetki gerekli</title></head>"
                  "<body style='font-family:system-ui,sans-serif;padding:24px'>"
                  "<h1>Yetki gerekli</h1><p>.bin y&uuml;klemek i&ccedil;in y&ouml;netim paneline giri&scaron; yap&#305;p "
                  "Yaz&#305;l&#305;m &rsaquo; &quot;.bin y&uuml;kle&quot; d&uuml;&#287;mesini kullan&#305;n "
                  "(ba&#287;lant&#305; 60 sn ge&ccedil;erlidir).</p><p><a href='/admin#ota'>Y&ouml;netim paneline git</a></p>"
                  "</body></html>"));
    return;
  }

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
  html += F("<form method='POST' action='/update?t=");
  html += ticket;  // auth_check_upload_ticket yalnizca 32 hex karakteri kabul eder
  html += F("' enctype='multipart/form-data'>"
            "<input type='file' name='firmware' accept='.bin,application/octet-stream' required>"
            "<button type='submit'>BIN YUKLE</button>"
            "</form><p class='muted'>Yukleme baglantisi 60 sn gecerlidir ve tek kullanimliktir; "
            "suresi dolarsa panelden tekrar acin.</p>"
            "<p class='muted'><a href='/admin#ota'>Admin panele don</a></p>"
            "</div></body></html>");

  noteHttpResponseSent();
  server.send(200, "text/html", html);
}

static void handleManualUpdateUpload() {
  noteWebActivity();

  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    resetManualOtaState();
    // Yetki: ?t= tek kullanimlik bilet (burada tuketilir) ya da gecerli Bearer oturumu.
    // WebServer, multipart govdesinden once URL argumanlarini ve basliklari ayristirir.
    bool ok = auth_check_upload_ticket(server.arg("t"), true) ||
              auth_check_bearer(server.header("Authorization"));
    if (!ok) {
      s_manualOta.lastError = "Yetkisiz yukleme: panelden yeniden '.bin yukle' ile acin";
      Serial.println("[OTA] Manual upload reddedildi: yetkisiz");
      return;  // active=false kalir; sonraki parcalar yok sayilir
    }
    s_manualOta.authorized = true;
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
  if (!s_manualOta.authorized) {
    String err = s_manualOta.lastError.length() ? s_manualOta.lastError : String("Yetkisiz yukleme");
    resetManualOtaState();
    server.send(401, "text/plain; charset=utf-8", err);
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
  s_manualOta.authorized = false;
  if (!s_manualOta.success) s_manualOta.lastError = "";
  noteHttpResponseSent();
  server.send(code, contentType, body);
}

static const char* computeAlarm(const PilotMeasurements& m, float iMax, bool staOk, uint32_t nowMs, int* level) {
  bool manualStopAlertOn = (g_manualStopAlertUntilMs != 0 && ((int32_t)(g_manualStopAlertUntilMs - nowMs) > 0));
  if (manualStopAlertOn) {
    *level = 2;
    return "Sarj manuel durduruldu";
  }
  if (m.stateStable == "E" || m.stateStable == "F") {
    *level = 2;
    return "Pilot hata durumu";
  }
  if (iMax > (g_currentLimitA + 1.0f)) {
    *level = 1;
    return "Akim limiti ustu";
  }
  if (!staOk) {
    *level = 1;
    return "Wi-Fi baglantisi yok";
  }
  *level = 0;
  return "Sistem normal";
}

// Kullanici ekrani ("/") icin herkese acik, salt-okunur, asgari alanli durum.
// Kalibrasyon, Wi-Fi profili, IP/MAC/host, OTA ve sifirlama bilgisi icermez.
static void handleStatusPublic() {
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
  float ia = 0.0f, ib = 0.0f, ic = 0.0f;
  updateDisplayCurrents(rawIa, rawIb, rawIc, &ia, &ib, &ic);
  float iMax = rawIa;
  if (rawIb > iMax) iMax = rawIb;
  if (rawIc > iMax) iMax = rawIc;
  float pW = accountingEnabled ? safeFinite(g_powerW) : 0.0f;
  bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
  int alarmLv = 0;
  const char* alarmTxt = computeAlarm(m, iMax, staOk, nowMs, &alarmLv);

  refreshDeviceIdentity();
  const String stationName = jsonEscape(String(s_stationLabel));
  const String stationAddr = jsonEscape(String(s_stationAddress));
  // Planli sarj: yalnizca yasakli mi ve ne zamana kadar (HH:MM).
  evse_sched::Eval schedEval = sched_eval();
  char schedUntil[6] = "";
  if (schedEval.blocked && schedEval.untilMin >= 0) evse_sched::format_hhmm((uint16_t)schedEval.untilMin, schedUntil);

  char* buf = s_jsonBuf;  // web gorevi tek is parcacigi; ortak tampon guvenli
  snprintf(
    buf, sizeof(s_jsonBuf),
    "{\"state\":\"%s\",\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,"
    "\"pW\":%.1f,\"eKWh\":%.3f,\"tSec\":%lu,\"phase\":%d,"
    "\"modeId\":%d,\"limitA\":%.1f,\"limitTargetA\":%.1f,\"staOk\":%d,"
    "\"stationName\":\"%s\",\"stationAddr\":\"%s\","
    "\"alarmLv\":%d,\"alarmTxt\":\"%s\","
    "\"sLive\":%d,\"sLiveSec\":%lu,\"sLiveKWh\":%.3f,"
    "\"schedBlocked\":%d,\"schedUntil\":\"%s\"}",
    m.stateStable.c_str(),
    ia, ib, ic,
    pW, safeFinite(g_energyKWh),
    (unsigned long)g_chargeSeconds,
    g_phaseCount,
    g_chargeMode,
    safeFinite(g_currentLimitA),
    safeFinite(g_targetCurrentLimitA),
    staOk ? 1 : 0,
    stationName.c_str(),
    stationAddr.c_str(),
    alarmLv,
    alarmTxt,
    g_sessionLive ? 1 : 0,
    (unsigned long)g_sessionLiveSeconds,
    safeFinite(g_sessionLiveEnergyKWh),
    schedEval.blocked ? 1 : 0,
    schedUntil
  );
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
  noteHttpResponseSent();
}

// Yonetim panelinin tam durum verisi (Bearer jeton ister).
static void handleStatus() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
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
  int ppRaw = analogRead(PP_ADC_PIN);
  float ppVolt = safeFinite((ppRaw * 3.3f) / 4095.0f);
  float cpHigh = safeFinite(m.cpHigh);
  float cpLow = safeFinite(m.cpLow);
  float adcHigh = safeFinite(m.adcHigh);
  float adcLow = safeFinite(m.adcLow);
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

  // Bulut ve planli sarj durumu (gizli anahtar icermez).
  char cloudJson[512];
  char schedJson[300];
  cloud_status_json(cloudJson, sizeof(cloudJson));
  sched_status_json(schedJson, sizeof(schedJson), true);
  if (cloudJson[0] == '\0') snprintf(cloudJson, sizeof(cloudJson), "null");
  if (schedJson[0] == '\0') snprintf(schedJson, sizeof(schedJson), "null");

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
    "\"rstTotal\":%lu,\"rstNow\":%lu,\"rstHist\":%lu,\"rstLastSec\":%lu,\"rstLastMode\":\"%s\",\"pwMustChange\":%d,"
    "\"cloud\":%s,\"sched\":%s}",
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
    resetModeLabel(s_resetLastModeId),
    auth_must_change_password() ? 1 : 0,
    cloudJson,
    schedJson
  );

  server.send(200, "application/json", s_jsonBuf);
  noteHttpResponseSent();
}

// Gecmis seanslar icin ayri JSON endpoint.
static void handleHistory() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
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
  noteWebActivity();
  if (!requireAdminAuth()) return;
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
  noteWebActivity();
  if (!requireAdminAuth()) return;
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
  noteWebActivity();
  if (!requireAdminAuth()) return;
  float calA = 0.0f, calB = 0.0f, calC = 0.0f;
  float offA = 0.0f, offB = 0.0f, offC = 0.0f;
  float rngLowMax = 0.0f, rngMidMax = 0.0f, rngLowOff = 0.0f, rngMidOff = 0.0f;
  bool locationChanged = false;
  bool stationNameChanged = false;
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
    float requested = clampFloatArg(server.arg("div"), 0.1f, 20.0f, CP_DIVIDER_RATIO);
    if (fabsf(requested - CP_DIVIDER_RATIO) > 0.02f) {
      pilot_set_divider(requested);
    }
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
  // Not: eski "userCss" parametresi artik kabul edilmez (yok sayilir).
  current_sensor_set_calibration(calA, calB, calC, offA, offB, offC);
  current_sensor_set_range_profile(rngLowMax, rngMidMax, rngLowOff, rngMidOff);
  if (locationChanged) {
    refreshStationAddress(true);
  }
  if (locationChanged || stationNameChanged) {
    saveLocationSettings();
  }
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handleRelay() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  if (server.hasArg("on")) relay_set(server.arg("on") == "1");
  server.send(200, "text/plain", "OK");
  noteHttpResponseSent();
}

static void handleRelayAuto() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
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

// ---- Panel girisi / oturum API'leri (bkz. auth.cpp) ----
static constexpr size_t kMaxAuthBodyLen = 512;

static void sendJsonNoStore(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

// JSON govdesini okur; cok buyuk / bozuk ise 400 gonderip false doner.
static bool readAuthJson(StaticJsonDocument<384>& doc) {
  const String& body = server.arg("plain");
  if (body.length() == 0 || body.length() > kMaxAuthBodyLen ||
      deserializeJson(doc, body) != DeserializationError::Ok) {
    sendJsonNoStore(400, "{\"error\":\"Gecersiz istek\"}");
    return false;
  }
  return true;
}

// POST /api/login {user,password} -> 200 {token,ttl,mustChange} | 401 {left} | 429 {retryAfter}
static void handleApiLogin() {
  noteWebActivity();
  StaticJsonDocument<384> doc;
  if (!readAuthJson(doc)) return;
  const char* user = doc["user"] | "";
  const char* pass = doc["password"] | "";
  char token[evse_auth::kTokenHexLen + 1] = {0};
  uint32_t info = 0;
  AuthLoginResult r = auth_login(user, pass, token, &info);
  char out[160];
  if (r == AuthLoginResult::Ok) {
    snprintf(out, sizeof(out), "{\"token\":\"%s\",\"ttl\":%lu,\"mustChange\":%d}",
             token, (unsigned long)auth_session_ttl_sec(), auth_must_change_password() ? 1 : 0);
    memset(token, 0, sizeof(token));
    sendJsonNoStore(200, out);
    noteHttpResponseSent();
    return;
  }
  if (r == AuthLoginResult::Locked) {
    server.sendHeader("Retry-After", String((unsigned long)info));
    snprintf(out, sizeof(out), "{\"retryAfter\":%lu}", (unsigned long)info);
    sendJsonNoStore(429, out);
    return;
  }
  snprintf(out, sizeof(out), "{\"left\":%lu}", (unsigned long)info);
  sendJsonNoStore(401, out);
}

// POST /api/logout (Bearer) -> 200; jeton gecersiz olsa da 200 doner.
static void handleApiLogout() {
  noteWebActivity();
  auth_logout(server.header("Authorization"));
  sendJsonNoStore(200, "{\"ok\":1}");
  noteHttpResponseSent();
}

// POST /api/password {current,new} (Bearer) -> 200 {token,ttl} | 403 {error} | 429 {error,retryAfter}
static void handleApiPassword() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  StaticJsonDocument<384> doc;
  if (!readAuthJson(doc)) return;
  const char* cur = doc["current"] | "";
  const char* next = doc["new"] | "";
  char token[evse_auth::kTokenHexLen + 1] = {0};
  uint32_t info = 0;
  AuthPwResult r = auth_change_password(cur, next, token, &info);
  char out[160];
  switch (r) {
    case AuthPwResult::Ok:
      snprintf(out, sizeof(out), "{\"ok\":1,\"token\":\"%s\",\"ttl\":%lu}",
               token, (unsigned long)auth_session_ttl_sec());
      memset(token, 0, sizeof(token));
      sendJsonNoStore(200, out);
      noteHttpResponseSent();
      return;
    case AuthPwResult::Locked:
      server.sendHeader("Retry-After", String((unsigned long)info));
      snprintf(out, sizeof(out), "{\"error\":\"Cok fazla hatali deneme. %lu sn sonra tekrar deneyin.\",\"retryAfter\":%lu}",
               (unsigned long)info, (unsigned long)info);
      sendJsonNoStore(429, out);
      return;
    case AuthPwResult::WrongCurrent:
      sendJsonNoStore(403, "{\"error\":\"Mevcut parola hatali.\"}");
      return;
    case AuthPwResult::TooShort:
      sendJsonNoStore(403, "{\"error\":\"Yeni parola en az 10 karakter olmali.\"}");
      return;
    case AuthPwResult::TooLong:
      sendJsonNoStore(403, "{\"error\":\"Yeni parola en fazla 64 karakter olabilir.\"}");
      return;
    case AuthPwResult::SameAsCurrent:
      sendJsonNoStore(403, "{\"error\":\"Yeni parola mevcut paroladan farkli olmali.\"}");
      return;
    case AuthPwResult::StoreFailed:
    default:
      sendJsonNoStore(500, "{\"error\":\"Parola kaydedilemedi (NVS).\"}");
      return;
  }
}

// POST /api/upload_ticket (Bearer) -> {t,ttl}: /update?t=... icin 60 sn'lik tek kullanimlik bilet.
static void handleApiUploadTicket() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  char ticket[evse_auth::kTokenHexLen + 1] = {0};
  auth_issue_upload_ticket(ticket);
  char out[96];
  snprintf(out, sizeof(out), "{\"t\":\"%s\",\"ttl\":%lu}", ticket,
           (unsigned long)(evse_auth::kUploadTicketTtlMs / 1000UL));
  sendJsonNoStore(200, out);
  noteHttpResponseSent();
}

// GET /api/cloud_cfg (Bearer) -> bulut durumu (gizli anahtar yok; yalnizca secretSet)
static void handleApiCloudCfgGet() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  char out[512];
  cloud_status_json(out, sizeof(out));
  sendJsonNoStore(200, out);
  noteHttpResponseSent();
}

// POST /api/cloud_cfg (Bearer). Alanlarin hepsi istege bagli:
// {on:bool, code:"ABC123"|"" (""=MAC varsayilani), url:"https://..."|"" (""=varsayilan),
//  secret:"<43 krk>", clearSecret:bool, requireCloudStart:bool} -> 200 durum | 400 {error}
static void handleApiCloudCfg() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  const String& body = server.arg("plain");
  StaticJsonDocument<512> doc;
  if (body.length() == 0 || body.length() > 512 || deserializeJson(doc, body) != DeserializationError::Ok ||
      !doc.is<JsonObject>()) {
    sendJsonNoStore(400, "{\"error\":\"Gecersiz istek\"}");
    return;
  }
  CloudCfgChange ch;
  char code[24] = "";
  char url[128] = "";
  char secret[64] = "";
  if (!doc["on"].isNull()) {
    if (!doc["on"].is<bool>()) { sendJsonNoStore(400, "{\"error\":\"on true/false olmali\"}"); return; }
    ch.on = doc["on"].as<bool>() ? 1 : 0;
  }
  if (!doc["requireCloudStart"].isNull()) {
    if (!doc["requireCloudStart"].is<bool>()) { sendJsonNoStore(400, "{\"error\":\"requireCloudStart true/false olmali\"}"); return; }
    ch.requireStart = doc["requireCloudStart"].as<bool>() ? 1 : 0;
  }
  if (!doc["code"].isNull()) {
    const char* c = doc["code"] | "";
    if (strlen(c) >= sizeof(code)) { sendJsonNoStore(400, "{\"error\":\"Istasyon kodu cok uzun\"}"); return; }
    size_t i = 0;
    for (; c[i]; ++i) code[i] = (char)toupper((unsigned char)c[i]);
    code[i] = '\0';
    ch.code = code;
  }
  if (!doc["url"].isNull()) {
    const char* u = doc["url"] | "";
    if (strlen(u) >= sizeof(url)) { sendJsonNoStore(400, "{\"error\":\"Sunucu adresi cok uzun\"}"); return; }
    snprintf(url, sizeof(url), "%s", u);
    ch.url = url;
  }
  if (!doc["secret"].isNull()) {
    const char* s = doc["secret"] | "";
    if (strlen(s) >= sizeof(secret)) { sendJsonNoStore(400, "{\"error\":\"Gizli anahtar 43 karakter olmali (A-Z a-z 0-9 _ -)\"}"); return; }
    snprintf(secret, sizeof(secret), "%s", s);
    ch.secret = secret;
  }
  ch.clearSecret = doc["clearSecret"] | false;
  const char* err = "";
  bool ok = cloud_apply_config(ch, &err);
  // Anahtar kopyalarini temizle (istek govdesi WebServer tarafinda istek bitince serbest kalir).
  memset(secret, 0, sizeof(secret));
  doc.clear();
  if (!ok) {
    String out = String("{\"error\":\"") + err + "\"}";
    sendJsonNoStore(400, out);
    return;
  }
  char out[512];
  cloud_status_json(out, sizeof(out));
  sendJsonNoStore(200, out);
  noteHttpResponseSent();
}

// GET /api/schedule (Bearer) -> planli sarj ayari + durum
// POST /api/schedule {on:bool, windows:[{s:"HH:MM",e:"HH:MM",d:1..127}]} (en cok 3) -> 200 durum | 400 {error}
static void handleApiScheduleGet() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  char out[300];
  sched_status_json(out, sizeof(out), true);
  sendJsonNoStore(200, out[0] ? out : "{}");
  noteHttpResponseSent();
}

static void handleApiSchedulePost() {
  noteWebActivity();
  if (!requireAdminAuth()) return;
  const String& body = server.arg("plain");
  StaticJsonDocument<768> doc;
  if (body.length() == 0 || body.length() > 512 || deserializeJson(doc, body) != DeserializationError::Ok) {
    sendJsonNoStore(400, "{\"error\":\"Gecersiz istek\"}");
    return;
  }
  evse_sched::Schedule s;
  JsonVariant onV = doc["on"];
  JsonArray arr = doc["windows"].as<JsonArray>();
  if (!onV.is<bool>() || arr.isNull()) {
    sendJsonNoStore(400, "{\"error\":\"on ve windows gerekli\"}");
    return;
  }
  if (arr.size() > evse_sched::kMaxWindows) {
    sendJsonNoStore(400, "{\"error\":\"En cok 3 aralik olabilir\"}");
    return;
  }
  s.enabled = onV.as<bool>();
  s.count = 0;
  for (JsonObject w : arr) {
    uint16_t a = 0, b = 0;
    int d = w["d"] | (int)evse_sched::kAllDays;
    if (!evse_sched::parse_hhmm(w["s"] | "", &a) || !evse_sched::parse_hhmm(w["e"] | "", &b) || d < 1 || d > 127) {
      sendJsonNoStore(400, "{\"error\":\"Saat HH:MM, gun 1-127 olmali\"}");
      return;
    }
    s.w[s.count].startMin = a;
    s.w[s.count].endMin = b;
    s.w[s.count].days = (uint8_t)d;
    s.count++;
  }
  const char* err = "";
  if (!sched_set(s, &err)) {
    String out = String("{\"error\":\"") + err + "\"}";
    sendJsonNoStore(400, out);
    return;
  }
  sched_loop();  // yeni ayari hemen degerlendir (durum yaniti guncel olsun)
  char out[300];
  sched_status_json(out, sizeof(out), true);
  sendJsonNoStore(200, out[0] ? out : "{}");
  noteHttpResponseSent();
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
  purgeLegacyUserCss();
  auth_init();
  setupWiFi();
  setupArduinoOta();
#if RELAY_USE_LATCH_PINS
  // Yalniz eski latch kart. Bu PCB'de GPIO7=L3, GPIO16=PP; cikisa cekilmez.
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
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/sw.js", HTTP_GET, handleServiceWorker);
  server.on("/app-icon.svg", HTTP_GET, handleAppIcon);
  server.on("/vehicle-top-art.svg", HTTP_GET, handleVehicleTopArt);
  server.on("/togg-arac-v2.webp", HTTP_GET, handleToggAracWebp);
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
  // Panel girisi (jeton tabanli). Yonetim veri uclari Bearer ister.
  server.on("/api/login", HTTP_POST, handleApiLogin);
  server.on("/api/logout", HTTP_POST, handleApiLogout);
  server.on("/api/password", HTTP_POST, handleApiPassword);
  server.on("/api/upload_ticket", HTTP_POST, handleApiUploadTicket);
  server.on("/api/cloud_cfg", HTTP_GET, handleApiCloudCfgGet);
  server.on("/api/cloud_cfg", HTTP_POST, handleApiCloudCfg);
  server.on("/api/schedule", HTTP_GET, handleApiScheduleGet);
  server.on("/api/schedule", HTTP_POST, handleApiSchedulePost);
  server.on("/status_public", HTTP_GET, handleStatusPublic);
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
  s_serverStarted = false;
  ensureServerStarted();
  if (s_webTaskHandle == nullptr) {
    BaseType_t taskOk = xTaskCreatePinnedToCore(
      web_task_runner,
      "webLoop",
      8192,  // /status ve bulut/zamanlayici JSON tamponlari icin
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
