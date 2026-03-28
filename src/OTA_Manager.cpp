#include "OTA_Manager.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>

namespace {
struct OtaContext {
  String manifestUrl;
  String sha1Fingerprint;
  uint32_t checkIntervalMs = 0;
  uint32_t lastCheckMs = 0;
  bool checkRequested = false;
  bool lastUpdateOk = false;
  bool pendingVerify = false;
  bool markedValid = false;
  uint32_t bootMs = 0;
} ctx;

HTTPUpdate httpUpdate;

constexpr uint32_t kVerifyDelayMs = 10 * 1000;   // Yeni firmware'i 10 sn ayakta tuttuktan sonra valid say.
constexpr uint32_t kVerifyTimeoutMs = 30 * 1000; // 30 sn icinde mark valid olmazsa bootloader rollback yapacak.

struct Manifest {
  String version;
  String url;
};

static bool wifiReady() {
  return WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0;
}

static void configureClient(WiFiClientSecure& client) {
  if (ctx.sha1Fingerprint.length() > 0) {
    client.setFingerprint(ctx.sha1Fingerprint.c_str());
  } else {
    client.setInsecure();
  }
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
  StaticJsonDocument<256> doc;
  auto err = deserializeJson(doc, payload);
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
  return out.version.length() > 0 && out.url.length() > 0;
}

static bool fetchManifest(Manifest& out) {
  if (ctx.manifestUrl.isEmpty()) {
    Serial.println("[OTA] Manifest URL tanimli degil");
    return false;
  }

  WiFiClientSecure client;
  configureClient(client);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(8000);

  Serial.printf("[OTA] Manifest GET: %s\n", ctx.manifestUrl.c_str());
  if (!http.begin(client, ctx.manifestUrl)) {
    Serial.println("[OTA] HTTP baslatilamadi");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] Manifest indirilemedi, HTTP %d\n", code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  return parseManifest(payload, out);
}

static bool performUpdate(const String& url) {
  WiFiClientSecure client;
  configureClient(client);

  httpUpdate.rebootOnUpdate(true);
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  t_httpUpdate_return ret = httpUpdate.update(client, url);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] Update HATA: %s (code %d)\n",
                    httpUpdate.getLastErrorString().c_str(),
                    httpUpdate.getLastError());
      return false;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] Yeni surum yok (HTTP_UPDATE_NO_UPDATES)");
      return true;
    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Update indirildi, reboot ediliyor..." );
      return true;
  }
  return false;
}

static void handleUpdateCheck() {
  if (!ctx.checkRequested && ctx.checkIntervalMs > 0 && (millis() - ctx.lastCheckMs) < ctx.checkIntervalMs) {
    return;
  }
  if (!wifiReady()) return;

  ctx.checkRequested = false;
  ctx.lastCheckMs = millis();

  Manifest m;
  if (!fetchManifest(m)) return;

  int cmp = compareVersionTokens(m.version, CURRENT_VERSION);
  Serial.printf("[OTA] Surumler: remote=%s local=%s cmp=%d\n", m.version.c_str(), CURRENT_VERSION, cmp);
  if (cmp > 0) {
    Serial.printf("[OTA] Yeni surum bulunuyor -> %s\n", m.url.c_str());
    ctx.lastUpdateOk = performUpdate(m.url);
  } else {
    Serial.println("[OTA] Cihaz guncel");
    ctx.lastUpdateOk = true;
  }
}

static void handleRollbackGuard() {
  if (!ctx.pendingVerify || ctx.markedValid) return;
  uint32_t now = millis();
  if (now - ctx.bootMs < kVerifyDelayMs) return; // biraz bekle

  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    ctx.markedValid = true;
    ctx.pendingVerify = false;
    Serial.println("[OTA] Yeni firmware stabil olarak isaretlendi; rollback iptal");
  } else {
    Serial.printf("[OTA] Rollback iptal basarisiz: %s\n", esp_err_to_name(err));
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

  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(running, &state);
  if (err == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    ctx.pendingVerify = true;
    Serial.println("[OTA] Pending verify: rollback penceresi acik");
  } else {
    ctx.pendingVerify = false;
  }
}

void loop() {
  handleRollbackGuard();
  handleUpdateCheck();
}

void triggerCheckNow() {
  ctx.checkRequested = true;
}

const char* currentVersion() {
  return CURRENT_VERSION;
}

bool lastUpdateSucceeded() {
  return ctx.lastUpdateOk;
}

}  // namespace OTA_Manager

