#include "OTA_Manager.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>
#include <mbedtls/sha256.h>

namespace {
struct OtaContext {
  String manifestUrl;
  String sha1Fingerprint;
  String lastRemoteVersion;
  String lastStatus;
  String lastError;
  uint32_t checkIntervalMs = 0;
  uint32_t lastCheckMs = 0;
  uint32_t deferUntilMs = 0;
  bool checkRequested = false;
  bool lastUpdateOk = false;
  bool pendingVerify = false;
  bool builtinPendingVerify = false;
  bool customTrialPending = false;
  bool markedValid = false;
  bool failureRecorded = false;
  uint32_t bootMs = 0;
  String verifyHeartbeatUrl;
  bool verifyHeartbeatResolved = false;
} ctx;

HTTPUpdate httpUpdate;
Preferences otaPrefs;

constexpr uint32_t kVerifyDelayMs = 10 * 1000;   // Yeni firmware'i 10 sn ayakta tuttuktan sonra valid say.
constexpr uint32_t kVerifyTimeoutMs = 30 * 1000; // 30 sn icinde mark valid olmazsa bootloader rollback yapacak.
constexpr uint8_t kBackoffStage1Tries = 5;
constexpr uint8_t kBackoffStage2Tries = 5;
constexpr uint32_t kBackoffStage1Ms = 5UL * 60UL * 1000UL;      // 5 dk
constexpr uint32_t kBackoffStage2Ms = 2UL * 60UL * 60UL * 1000UL; // 2 saat
constexpr uint32_t kBackoffStage3Ms = 3UL * 60UL * 60UL * 1000UL; // 3 saat
constexpr uint32_t kUptimePersistIntervalMs = 60UL * 1000UL;
constexpr uint8_t kTrialSubtypeUnset = 0xFF;

struct RetryState {
  uint32_t uptimeBaseMs = 0;
  uint32_t nextAllowedMs = 0;
  uint8_t failCount = 0;
  uint32_t lastPersistMs = 0;
  bool prefsReady = false;
} retry;

struct Manifest {
  String version;
  String url;
  String healthUrl;
  String sha256;
  uint32_t size = 0;
};

static const char* partitionSubtypeLabel(esp_partition_subtype_t subtype) {
  switch (subtype) {
    case ESP_PARTITION_SUBTYPE_APP_FACTORY:
      return "factory";
    case ESP_PARTITION_SUBTYPE_APP_OTA_0:
      return "ota_0";
    case ESP_PARTITION_SUBTYPE_APP_OTA_1:
      return "ota_1";
    default:
      return "other";
  }
}

static const char* imageStateLabel(esp_ota_img_states_t state) {
  switch (state) {
    case ESP_OTA_IMG_NEW:
      return "new";
    case ESP_OTA_IMG_PENDING_VERIFY:
      return "pending_verify";
    case ESP_OTA_IMG_VALID:
      return "valid";
    case ESP_OTA_IMG_INVALID:
      return "invalid";
    case ESP_OTA_IMG_ABORTED:
      return "aborted";
    case ESP_OTA_IMG_UNDEFINED:
      return "undefined";
    default:
      return "unknown";
  }
}

static void syncPendingVerifyFlag() {
  ctx.pendingVerify = ctx.builtinPendingVerify || ctx.customTrialPending;
}

static bool wifiReady() {
  return WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0;
}

static uint32_t virtualMillis() {
  return retry.uptimeBaseMs + millis();
}

static void persistUptimeNow() {
  if (!retry.prefsReady) return;
  otaPrefs.putUInt("uptime_ms", virtualMillis());
  retry.lastPersistMs = millis();
}

static void persistUptimeIfNeeded() {
  if (!retry.prefsReady) return;
  uint32_t now = millis();
  if (now - retry.lastPersistMs < kUptimePersistIntervalMs) return;
  persistUptimeNow();
}

static void loadRetryState() {
  if (!otaPrefs.begin("ota_guard", false)) return;
  retry.prefsReady = true;
  retry.uptimeBaseMs = otaPrefs.getUInt("uptime_ms", 0);
  retry.nextAllowedMs = otaPrefs.getUInt("next_ms", 0);
  retry.failCount = otaPrefs.getUChar("fail_cnt", 0);
  retry.lastPersistMs = millis();

  uint32_t now = virtualMillis();
  if (retry.nextAllowedMs != 0 && (int32_t)(now - retry.nextAllowedMs) >= 0) {
    retry.nextAllowedMs = 0;
    otaPrefs.putUInt("next_ms", 0);
  }
}

static void clearCustomTrialState(bool clearRuntime = true) {
  if (retry.prefsReady) {
    otaPrefs.putUChar("trial_on", 0);
    otaPrefs.putString("trial_ver", "");
    otaPrefs.putString("trial_hb", "");
    otaPrefs.putUChar("trial_sub", kTrialSubtypeUnset);
    persistUptimeNow();
  }
  if (clearRuntime) {
    ctx.customTrialPending = false;
    syncPendingVerifyFlag();
  }
}

static bool armCustomTrialState(const Manifest& manifest, const esp_partition_t* target) {
  if (!retry.prefsReady || target == nullptr) {
    Serial.println("[OTA] Custom trial state kaydedilemedi; Preferences hazir degil");
    return false;
  }

  String heartbeat = manifest.healthUrl.length() ? manifest.healthUrl : ctx.manifestUrl;
  otaPrefs.putUChar("trial_on", 1);
  otaPrefs.putString("trial_ver", manifest.version);
  otaPrefs.putString("trial_hb", heartbeat);
  otaPrefs.putUChar("trial_sub", static_cast<uint8_t>(target->subtype));
  persistUptimeNow();
  Serial.printf("[OTA] Custom trial armed: version=%s target=%s\n",
                manifest.version.c_str(),
                partitionSubtypeLabel(target->subtype));
  return true;
}

static void resumeCustomTrialState() {
  if (!retry.prefsReady) return;
  if (otaPrefs.getUChar("trial_on", 0) == 0) return;

  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) {
    clearCustomTrialState();
    return;
  }

  String armedVersion = otaPrefs.getString("trial_ver", "");
  String heartbeat = otaPrefs.getString("trial_hb", "");
  uint8_t armedSubtype = otaPrefs.getUChar("trial_sub", kTrialSubtypeUnset);
  if (armedVersion.length() == 0) {
    clearCustomTrialState();
    return;
  }

  bool versionMatches = (armedVersion == CURRENT_VERSION);
  bool partitionMatches = (armedSubtype == kTrialSubtypeUnset) ||
                          (armedSubtype == static_cast<uint8_t>(running->subtype));

  if (versionMatches && partitionMatches) {
    ctx.customTrialPending = true;
    syncPendingVerifyFlag();
    if (heartbeat.length()) {
      ctx.verifyHeartbeatUrl = heartbeat;
      ctx.verifyHeartbeatResolved = true;
    }
    Serial.printf("[OTA] Custom trial aktif: version=%s partition=%s\n",
                  armedVersion.c_str(),
                  partitionSubtypeLabel(running->subtype));
    return;
  }

  if (running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY || !versionMatches) {
    Serial.printf("[OTA] Stale custom trial temizleniyor: armed=%s current=%s running=%s\n",
                  armedVersion.c_str(),
                  CURRENT_VERSION,
                  partitionSubtypeLabel(running->subtype));
    clearCustomTrialState();
  }
}

static uint32_t computeBackoffMs(uint8_t failCount) {
  if (failCount <= kBackoffStage1Tries) return kBackoffStage1Ms;
  if (failCount <= (kBackoffStage1Tries + kBackoffStage2Tries)) return kBackoffStage2Ms;
  return kBackoffStage3Ms;
}

static bool backoffActive() {
  if (!retry.prefsReady) return false;
  if (retry.nextAllowedMs == 0) return false;
  return (int32_t)(virtualMillis() - retry.nextAllowedMs) < 0;
}

static void setBackoffStatus() {
  if (!backoffActive()) return;
  uint32_t now = virtualMillis();
  uint32_t remain = retry.nextAllowedMs - now;
  uint32_t minutes = (remain + 60000UL - 1UL) / 60000UL;
  ctx.lastStatus = "backoff";
  ctx.lastError = String("OTA beklemede, ") + String(minutes) + " dk sonra denenecek";
}

static void noteFailure(const char* reason) {
  if (!retry.prefsReady) return;
  if (retry.failCount < 250) retry.failCount++;
  uint32_t delay = computeBackoffMs(retry.failCount);
  retry.nextAllowedMs = virtualMillis() + delay;
  otaPrefs.putUChar("fail_cnt", retry.failCount);
  otaPrefs.putUInt("next_ms", retry.nextAllowedMs);
  persistUptimeNow();

  ctx.lastStatus = "backoff";
  String msg = "OTA beklemeye alindi, ";
  msg += String((delay + 60000UL - 1UL) / 60000UL);
  msg += " dk sonra tekrar denenecek";
  if (reason && reason[0]) {
    msg += " (";
    msg += reason;
    msg += ")";
  }
  ctx.lastError = msg;
}

static void clearBackoff() {
  if (!retry.prefsReady) return;
  retry.failCount = 0;
  retry.nextAllowedMs = 0;
  otaPrefs.putUChar("fail_cnt", 0);
  otaPrefs.putUInt("next_ms", 0);
  persistUptimeNow();
}

static void configureClient(WiFiClientSecure& client) {
  // Development: setInsecure() kullan. Produksiyonda sertifika ekle:
  // client.setSignedCertificate(x509);
  // SHA1 fingerprint desteği: 
  //   static const char* FP = "AA:BB:CC:DD:...";
  //   const uint8_t fp[] = {0xAA, 0xBB, 0xCC, ...};
  //   client.setFingerprint(fp);
  client.setInsecure();
}

static int compareVersionTokens(const String& a, const String& b) {
  int aParts[4] = {0}, bParts[4] = {0};
  auto fill = [](const String& src, int (&dst)[4]) {
    int partIdx = 0;
    int value = 0;
    for (size_t i = 0; i <= src.length() && partIdx < 4; ++i) {
      char c = (i < src.length()) ? src[i] : '.'; // sentinel
      if (c == '.') {
        dst[partIdx++] = value;
        value = 0;
      } else if (c >= '0' && c <= '9') {
        value = value * 10 + (c - '0');
      } else {
        // alfanumerik disi karakteri sifirla
        value = 0;
      }
    }
  };
  fill(a, aParts);
  fill(b, bParts);
  for (int i = 0; i < 4; ++i) {
    if (aParts[i] > bParts[i]) return 1;
    if (aParts[i] < bParts[i]) return -1;
  }
  return 0;
}

static bool parseManifest(const String& payload, Manifest& out) {
  String cleanPayload = payload;
  cleanPayload.trim();
  if (cleanPayload.length() >= 3 &&
      (uint8_t)cleanPayload[0] == 0xEF &&
      (uint8_t)cleanPayload[1] == 0xBB &&
      (uint8_t)cleanPayload[2] == 0xBF) {
    cleanPayload.remove(0, 3);
  }

  StaticJsonDocument<256> doc;
  auto err = deserializeJson(doc, cleanPayload);
  if (err) {
    Serial.printf("[OTA] version.json parse hatasi: %s\n", err.c_str());
    return false;
  }
  if (!doc.containsKey("version") || !doc.containsKey("url")) {
    Serial.println("[OTA] version.json beklenen alanlari icermiyor (version, url)");
    return false;
  }
  out.version = doc["version"].as<String>();
  out.url = doc["url"].as<String>();
  if (doc.containsKey("health_url")) out.healthUrl = doc["health_url"].as<String>();
  if (doc.containsKey("sha256")) out.sha256 = doc["sha256"].as<String>();
  if (doc.containsKey("size")) out.size = doc["size"].as<uint32_t>();
  return out.version.length() > 0 && out.url.length() > 0;
}

static bool fetchManifest(Manifest& out) {
  if (ctx.manifestUrl.isEmpty()) {
    ctx.lastStatus = "manifest_url_missing";
    ctx.lastError = "Manifest URL tanimli degil";
    Serial.println("[OTA] Manifest URL tanimli degil");
    return false;
  }

  WiFiClientSecure client;
  configureClient(client);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(3000);

  Serial.printf("[OTA] Manifest GET: %s\n", ctx.manifestUrl.c_str());
  if (!http.begin(client, ctx.manifestUrl)) {
    ctx.lastStatus = "manifest_begin_failed";
    ctx.lastError = "HTTP baslatilamadi";
    Serial.println("[OTA] HTTP baslatilamadi");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    ctx.lastStatus = "manifest_http_error";
    ctx.lastError = String("Manifest HTTP ") + code;
    Serial.printf("[OTA] Manifest indirilemedi, HTTP %d\n", code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  if (!parseManifest(payload, out)) {
    ctx.lastStatus = "manifest_parse_failed";
    ctx.lastError = "Manifest parse hatasi";
    return false;
  }
  ctx.lastRemoteVersion = out.version;
  ctx.lastStatus = "manifest_ok";
  ctx.lastError = "";
  return true;
}

static bool performUpdate(const Manifest& manifest) {
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
  if (!next ||
      (next->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_0 &&
       next->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_1)) {
    ctx.lastStatus = "update_target_invalid";
    ctx.lastError = "OTA hedef partition guvenli degil (yalnizca ota_0/ota_1 izinli)";
    Serial.println("[OTA] RED: OTA sadece ota_0/ota_1 partitionlarina yazabilir");
    return false;
  }

  if (!armCustomTrialState(manifest, next)) {
    ctx.lastStatus = "trial_state_error";
    ctx.lastError = "OTA deneme durumu kaydedilemedi";
    return false;
  }

  WiFiClientSecure client;
  configureClient(client);

  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(client, manifest.url);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      clearCustomTrialState(false);
      ctx.lastStatus = "update_failed";
      ctx.lastError = httpUpdate.getLastErrorString();
      Serial.printf("[OTA] Update HATA: %s (code %d)\n",
                    httpUpdate.getLastErrorString().c_str(),
                    httpUpdate.getLastError());
      return false;
    case HTTP_UPDATE_NO_UPDATES:
      clearCustomTrialState(false);
      ctx.lastStatus = "no_updates";
      ctx.lastError = "";
      Serial.println("[OTA] Yeni surum yok (HTTP_UPDATE_NO_UPDATES)");
      return true;
    case HTTP_UPDATE_OK:
      ctx.lastStatus = "update_ok";
      ctx.lastError = "";
      Serial.println("[OTA] Update indirildi, reboot ediliyor..." );
      return true;
  }
  return false;
}

static bool switchBootToFactory() {
  const esp_partition_t* factory =
      esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr);
  if (!factory) {
    Serial.println("[OTA] Factory partition bulunamadi");
    return false;
  }
  esp_err_t err = esp_ota_set_boot_partition(factory);
  if (err != ESP_OK) {
    Serial.printf("[OTA] Factory boot secilemedi: %s\n", esp_err_to_name(err));
    return false;
  }
  Serial.println("[OTA] Bir sonraki boot: FACTORY");
  return true;
}

static String toLowerHex(const uint8_t* buf, size_t len) {
  static const char* kHex = "0123456789abcdef";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out += kHex[(buf[i] >> 4) & 0x0F];
    out += kHex[buf[i] & 0x0F];
  }
  return out;
}

static bool precheckFirmware(const Manifest& m) {
  if (m.size == 0 && m.sha256.length() == 0) {
    ctx.lastStatus = "precheck_skipped";
    return true;
  }

  WiFiClientSecure client;
  configureClient(client);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(5000);
  if (!http.begin(client, m.url)) {
    ctx.lastStatus = "precheck_begin_failed";
    ctx.lastError = "OTA precheck HTTP baslatilamadi";
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    ctx.lastStatus = "precheck_http_error";
    ctx.lastError = String("OTA precheck HTTP ") + code;
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  mbedtls_sha256_context shaCtx;
  mbedtls_sha256_init(&shaCtx);
  mbedtls_sha256_starts_ret(&shaCtx, 0);

  uint8_t buf[1024];
  uint32_t total = 0;
  uint32_t lastDataMs = millis();
  while (http.connected()) {
    size_t avail = stream->available();
    if (avail == 0) {
      if (millis() - lastDataMs > 8000) break;
      delay(1);
      continue;
    }
    if (avail > sizeof(buf)) avail = sizeof(buf);
    int r = stream->readBytes(reinterpret_cast<char*>(buf), avail);
    if (r <= 0) continue;
    total += static_cast<uint32_t>(r);
    mbedtls_sha256_update_ret(&shaCtx, buf, static_cast<size_t>(r));
    lastDataMs = millis();
  }

  uint8_t digest[32];
  mbedtls_sha256_finish_ret(&shaCtx, digest);
  mbedtls_sha256_free(&shaCtx);
  http.end();

  if (m.size != 0 && total != m.size) {
    ctx.lastStatus = "precheck_size_mismatch";
    ctx.lastError = String("OTA size mismatch exp=") + m.size + " got=" + total;
    return false;
  }

  if (m.sha256.length() > 0) {
    String expected = m.sha256;
    expected.toLowerCase();
    String actual = toLowerHex(digest, sizeof(digest));
    if (actual != expected) {
      ctx.lastStatus = "precheck_hash_mismatch";
      ctx.lastError = "OTA SHA256 mismatch";
      return false;
    }
  }

  ctx.lastStatus = "precheck_ok";
  ctx.lastError = "";
  return true;
}

static bool sendHeartbeat(const String& url) {
  if (url.length() == 0) return false;
  WiFiClientSecure client;
  configureClient(client);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(2500);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  http.end();
  return code >= 200 && code < 300;
}

static bool markRunningAppValid() {
  if (ctx.builtinPendingVerify) {
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
      Serial.printf("[OTA] Rollback iptal basarisiz: %s\n", esp_err_to_name(err));
      return false;
    }
  }

  clearCustomTrialState(false);
  ctx.markedValid = true;
  ctx.builtinPendingVerify = false;
  ctx.customTrialPending = false;
  syncPendingVerifyFlag();
  clearBackoff();
  Serial.println("[OTA] Yeni firmware stabil olarak isaretlendi; rollback iptal");
  return true;
}

static void handleUpdateCheck() {
  if (!ctx.checkRequested && ctx.checkIntervalMs > 0 && (millis() - ctx.lastCheckMs) < ctx.checkIntervalMs) {
    return;
  }
  if (!ctx.checkRequested && ctx.deferUntilMs != 0 && (int32_t)(ctx.deferUntilMs - millis()) > 0) {
    return;
  }
  if (backoffActive()) {
    setBackoffStatus();
    return;
  }
  if (!wifiReady()) return;

  ctx.checkRequested = false;
  ctx.lastCheckMs = millis();
  ctx.lastStatus = "checking";
  ctx.lastError = "";

  Manifest m;
  if (!fetchManifest(m)) return;

  int cmp = compareVersionTokens(m.version, CURRENT_VERSION);
  Serial.printf("[OTA] Surumler: remote=%s local=%s cmp=%d\n", m.version.c_str(), CURRENT_VERSION, cmp);
  if (cmp > 0) {
    ctx.lastStatus = "update_available";
    Serial.printf("[OTA] Yeni surum bulunuyor -> %s\n", m.url.c_str());
    if (!precheckFirmware(m)) {
      ctx.lastUpdateOk = false;
      noteFailure("precheck_failed");
      return;
    }
    ctx.lastUpdateOk = performUpdate(m);
    if (!ctx.lastUpdateOk) {
      noteFailure("update_failed");
    }
  } else {
    ctx.lastStatus = "up_to_date";
    Serial.println("[OTA] Cihaz guncel");
    ctx.lastUpdateOk = true;
    clearBackoff();
  }
}

static void handleRollbackGuard() {
  if (!ctx.pendingVerify || ctx.markedValid) return;
  uint32_t now = millis();
  if (now - ctx.bootMs < kVerifyDelayMs) return; // biraz bekle

  if (!ctx.verifyHeartbeatResolved) {
    Manifest m;
    bool manifestOk = fetchManifest(m);
    ctx.verifyHeartbeatUrl = (manifestOk && m.healthUrl.length()) ? m.healthUrl : ctx.manifestUrl;
    ctx.verifyHeartbeatResolved = true;
  }

  if (wifiReady() && sendHeartbeat(ctx.verifyHeartbeatUrl)) {
    markRunningAppValid();
    return;
  }

  if ((now - ctx.bootMs) > kVerifyTimeoutMs) {
    ctx.lastStatus = "healthcheck_timeout_factory";
    ctx.lastError = "30 sn onay yok, factory partitiona donuluyor";
    Serial.println("[OTA] 30 sn icinde onay yok; FACTORY fallback tetikleniyor");
    if (!ctx.failureRecorded) {
      noteFailure("healthcheck_timeout");
      ctx.failureRecorded = true;
    }
    if (switchBootToFactory()) {
      clearCustomTrialState();
      persistUptimeNow();
      delay(100);
      ESP.restart();
    }
    // Factory secimi basarisizsa klasik rollback fallback
    clearCustomTrialState();
    esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    if (err != ESP_OK) {
      Serial.printf("[OTA] rollback_and_reboot basarisiz: %s\n", esp_err_to_name(err));
      ESP.restart();
    }
  }

  if (!ctx.markedValid && (now - ctx.bootMs) > kVerifyTimeoutMs) {
    Serial.println("[OTA] 30 sn icinde dogrulanamadi; reset olursa bootloader onceki bolume donecek");
  }
}

}  // namespace

namespace OTA_Manager {

void begin(const char* manifestUrl, uint32_t checkIntervalMs, const char* sha1Fingerprint) {
  ctx.manifestUrl = manifestUrl ? manifestUrl : "";
  ctx.checkIntervalMs = checkIntervalMs;
  ctx.checkRequested = true;
  ctx.sha1Fingerprint = sha1Fingerprint ? sha1Fingerprint : "";
  ctx.bootMs = millis();
  ctx.lastUpdateOk = false;
  ctx.markedValid = false;
  ctx.verifyHeartbeatResolved = false;
  ctx.verifyHeartbeatUrl = "";
  ctx.lastRemoteVersion = "-";
  ctx.lastStatus = "boot";
  ctx.lastError = "";
  ctx.failureRecorded = false;

  loadRetryState();
  resumeCustomTrialState();

  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(running, &state);
  if (err == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    ctx.builtinPendingVerify = true;
    syncPendingVerifyFlag();
    Serial.println("[OTA] Pending verify: rollback penceresi acik");
  } else {
    ctx.builtinPendingVerify = false;
    syncPendingVerifyFlag();
  }
}

void loop() {
  handleRollbackGuard();
  handleUpdateCheck();
  persistUptimeIfNeeded();
}

void triggerCheckNow() {
  ctx.checkRequested = true;
}

void deferPeriodicChecks(uint32_t ms) {
  uint32_t until = millis() + ms;
  if ((int32_t)(until - ctx.deferUntilMs) > 0) {
    ctx.deferUntilMs = until;
  }
}

const char* currentVersion() {
  return CURRENT_VERSION;
}

bool lastUpdateSucceeded() {
  return ctx.lastUpdateOk;
}

const char* lastRemoteVersion() {
  return ctx.lastRemoteVersion.c_str();
}

const char* lastStatusText() {
  return ctx.lastStatus.c_str();
}

const char* lastErrorText() {
  return ctx.lastError.c_str();
}

uint32_t lastCheckAgeMs() {
  if (ctx.lastCheckMs == 0) return 0;
  return millis() - ctx.lastCheckMs;
}

const char* runningPartitionLabel() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return "missing";
  return partitionSubtypeLabel(running->subtype);
}

const char* runningImageStateLabel() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return "partition_missing";

  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(running, &state);
  if (err != ESP_OK) return "state_unavailable";
  return imageStateLabel(state);
}

}  // namespace OTA_Manager
