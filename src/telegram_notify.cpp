#include "telegram_notify.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_system.h>
#include <Preferences.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#include "app_config.h"
#include "OTA_Manager.h"
#include "app_log.h"

#ifndef TELEGRAM_BOT_TOKEN
#define TELEGRAM_BOT_TOKEN ""
#endif

#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID ""
#endif

enum PendingAction : uint8_t {
  kActionNone = 0,
  kActionRestart,
  kActionRollback
};

static bool s_notified = false;
static long s_lastUpdateId = 0;
static char s_lastNotifiedState[4] = "";
static bool s_uidLoaded = false;
static volatile bool s_uidDirty = false;
static TaskHandle_t s_tgTaskHandle = NULL;

static char s_pendingCmd[96] = {0};
static volatile bool s_cmdReady = false;
static volatile uint8_t s_pendingAction = kActionNone;

static bool s_wifiWasConnected = false;
static uint32_t s_wifiDownSinceMs = 0;

static constexpr uint8_t kOutQueueSize = 6;
static constexpr size_t kOutMsgMax = 360;
static char s_outQueue[kOutQueueSize][kOutMsgMax];
static uint8_t s_outHead = 0;
static uint8_t s_outCount = 0;
static portMUX_TYPE s_tgMux = portMUX_INITIALIZER_UNLOCKED;

static void saveUpdateId(long uid) {
  Preferences prefs;
  if (prefs.begin("tgbot", false)) {
    prefs.putLong("last_uid", uid);
    prefs.end();
  }
}

static void loadUpdateId() {
  Preferences prefs;
  if (prefs.begin("tgbot", true)) {
    s_lastUpdateId = prefs.getLong("last_uid", 0);
    prefs.end();
  }
  Serial.printf("[TG] Son update_id: %ld\n", s_lastUpdateId);
}

static String displayName() {
  if (g_boardCustomName.length() > 0) return g_boardCustomName;
  return g_boardId + " (" + g_boardName + ")";
}

static String urlEncode(const String& s) {
  String out;
  out.reserve(s.length() * 3);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' ||
        c == '.' || c == '~' || c == ' ') {
      out += (c == ' ') ? '+' : c;
    } else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", (uint8_t)c);
      out += buf;
    }
  }
  return out;
}

static bool enqueueMessage(const String& text) {
  if (text.length() == 0) return false;
  portENTER_CRITICAL(&s_tgMux);
  uint8_t idx;
  if (s_outCount < kOutQueueSize) {
    idx = (s_outHead + s_outCount) % kOutQueueSize;
    s_outCount++;
  } else {
    idx = s_outHead;
    s_outHead = (s_outHead + 1) % kOutQueueSize;
  }
  strncpy(s_outQueue[idx], text.c_str(), kOutMsgMax - 1);
  s_outQueue[idx][kOutMsgMax - 1] = '\0';
  portEXIT_CRITICAL(&s_tgMux);
  return true;
}

static bool outQueueIsEmpty() {
  portENTER_CRITICAL(&s_tgMux);
  bool empty = (s_outCount == 0);
  portEXIT_CRITICAL(&s_tgMux);
  return empty;
}

static bool dequeueMessage(char* out, size_t outSize) {
  portENTER_CRITICAL(&s_tgMux);
  if (s_outCount == 0) {
    portEXIT_CRITICAL(&s_tgMux);
    return false;
  }
  strncpy(out, s_outQueue[s_outHead], outSize - 1);
  out[outSize - 1] = '\0';
  s_outHead = (s_outHead + 1) % kOutQueueSize;
  s_outCount--;
  portEXIT_CRITICAL(&s_tgMux);
  return true;
}

static void sendTelegramMessage(const char* text) {
  if (text == nullptr || text[0] == '\0') return;
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;

  String url = "https://api.telegram.org/bot";
  url += TELEGRAM_BOT_TOKEN;
  url += "/sendMessage?chat_id=";
  url += TELEGRAM_CHAT_ID;
  url += "&text=";
  url += urlEncode(String(text));

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(2000);
  if (http.begin(client, url)) {
    int code = http.GET();
    if (code != 200) {
      Serial.printf("[TG] sendMessage hatasi: HTTP %d\n", code);
    }
    http.end();
  }
}

static void stateLabel(const char* st, String& name, String& desc) {
  if (strcmp(st, "A") == 0) { name = "Hazır"; desc = "Araç bekleniyor"; }
  else if (strcmp(st, "B") == 0) { name = "Bağlı"; desc = "Araç bağlandı, hazır bekliyor"; }
  else if (strcmp(st, "C") == 0) { name = "Şarj Ediliyor"; desc = "Enerji aktarımı sürüyor"; }
  else if (strcmp(st, "D") == 0) { name = "Havalandırma"; desc = "Şarj sürüyor (fan/havalandırma)"; }
  else if (strcmp(st, "E") == 0) { name = "Şarj Hatası"; desc = "Pilot hata durumu"; }
  else if (strcmp(st, "F") == 0) { name = "Kritik Hata"; desc = "Koruma aktif, şarj durduruldu"; }
  else { name = st; desc = "Bilinmeyen durum"; }
}

static String formatUptime() {
  uint32_t sec = millis() / 1000;
  uint32_t d = sec / 86400;
  uint32_t h = (sec % 86400) / 3600;
  uint32_t m = (sec % 3600) / 60;
  uint32_t s = sec % 60;
  String out;
  if (d > 0) out += String(d) + "g ";
  if (h > 0 || d > 0) out += String(h) + "s ";
  out += String(m) + "dk " + String(s) + "sn";
  return out;
}

static String myStationCode() {
  if (g_boardMac.length() < 17) return "";
  String tail = g_boardMac.substring(12);
  tail.replace(":", "");
  tail.toUpperCase();
  return tail;
}

static bool isValidStationCode(const String& code) {
  if (code.length() != 6) return false;
  for (size_t i = 0; i < code.length(); ++i) {
    char c = code[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
  }
  return true;
}

static String normalizeTr(const String& in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length();) {
    uint8_t c = (uint8_t)in[i];
    if (c == 0xC3 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0xA7 || c2 == 0x87) { out += 'c'; i += 2; continue; }
      if (c2 == 0xB6 || c2 == 0x96) { out += 'o'; i += 2; continue; }
      if (c2 == 0xBC || c2 == 0x9C) { out += 'u'; i += 2; continue; }
    } else if (c == 0xC4 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0x9F || c2 == 0x9E) { out += 'g'; i += 2; continue; }
      if (c2 == 0xB1 || c2 == 0xB0) { out += 'i'; i += 2; continue; }
    } else if (c == 0xC5 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0x9F || c2 == 0x9E) { out += 's'; i += 2; continue; }
    }
    char ascii = (char)c;
    if (ascii >= 'A' && ascii <= 'Z') ascii = ascii - 'A' + 'a';
    out += ascii;
    i++;
  }
  return out;
}

static int matchMyNamePrefix(const String& normMsg) {
  String base[3];
  base[0] = normalizeTr(displayName());
  base[1] = normalizeTr(g_boardCustomName);
  base[2] = normalizeTr(g_boardName);

  String candidates[6];
  for (int i = 0; i < 3; i++) {
    candidates[i] = base[i];
    String nospace = base[i];
    nospace.replace(" ", "");
    candidates[3 + i] = nospace;
  }

  int best = 0;
  for (int i = 0; i < 6; i++) {
    const String& cand = candidates[i];
    if (cand.length() == 0) continue;
    if (normMsg == cand) {
      if ((int)cand.length() > best) best = cand.length();
    } else if (normMsg.startsWith(cand + " ")) {
      if ((int)cand.length() > best) best = cand.length();
    }
  }
  return best;
}

static void handleCommand(const String& rawCmd, const String& pilotState) {
  String cmd = rawCmd;
  cmd.trim();
  String norm = normalizeTr(cmd);

  String target = "";
  if (norm.startsWith("/")) {
    if (norm.indexOf(' ') > 1) {
      String firstToken = norm.substring(1, norm.indexOf(' '));
      firstToken.trim();
      if (firstToken == "all") {
        target = "ALL";
        cmd = "/" + cmd.substring(cmd.indexOf(' ') + 1);
        cmd.trim();
        norm = "/" + norm.substring(norm.indexOf(' ') + 1);
        norm.trim();
      } else if (isValidStationCode(firstToken)) {
        target = firstToken;
        cmd = "/" + cmd.substring(cmd.indexOf(' ') + 1);
        cmd.trim();
        norm = "/" + norm.substring(norm.indexOf(' ') + 1);
        norm.trim();
      }
    }
  } else {
    int sp = norm.indexOf(' ');
    String firstTok = (sp > 0) ? norm.substring(0, sp) : "";
    String rest = (sp > 0) ? norm.substring(sp + 1) : "";
    rest.trim();

    int nameLen = matchMyNamePrefix(norm);
    if (nameLen > 0) {
      target = myStationCode();
      cmd = norm.substring(nameLen);
      cmd.trim();
      norm = cmd;
    } else if (firstTok == "hepsi" || firstTok == "tumu" || firstTok == "herkes") {
      target = "ALL";
      cmd = rest;
      norm = rest;
    } else if (isValidStationCode(firstTok)) {
      target = firstTok;
      cmd = rest;
      norm = rest;
    } else {
      cmd = norm;
    }
  }

  const String myCode = myStationCode();
  if (target.length() > 0 && target != "ALL" && target != myCode) {
    Serial.printf("[TG] Komut baska karta ait (%s), yok sayildi\n", target.c_str());
    return;
  }

  bool needsTarget =
    norm.startsWith("durum") || norm.startsWith("bilgi") ||
    norm.startsWith("guncelle") || norm.startsWith("geri al") ||
    norm.startsWith("gerial") || norm.startsWith("yeniden baslat") ||
    norm.startsWith("isim") || norm.startsWith("log") || norm.startsWith("kayit") ||
    norm.startsWith("/status") || norm.startsWith("/info") ||
    norm.startsWith("/update") || norm.startsWith("/rollback") ||
    norm.startsWith("/restart") || norm.startsWith("/name") ||
    norm.startsWith("/log");
  if (needsTarget && target.length() == 0) {
    enqueueMessage(
      displayName() + "\n"
      "Birden fazla istasyon ayni bota bagli.\n"
      "Komutu kart adiyla gonderin, orn:\n" +
      displayName() + " durum\n"
      "Tum kartlara: hepsi durum");
    return;
  }

  if (cmd.startsWith("/name") || norm.startsWith("isim ")) {
    String arg;
    if (cmd.startsWith("/name")) {
      arg = cmd.substring(5);
    } else {
      arg = norm.substring(5);
    }
    arg.trim();
    if (arg.length() == 0) {
      enqueueMessage("Kart ismi: " + displayName() + "\nMAC: " + g_boardMac +
                     "\nDegistirmek icin: " + displayName() + " isim YeniIsim");
    } else if (arg.length() > 30) {
      enqueueMessage("Isim cok uzun (max 30 karakter)");
    } else {
      g_boardCustomName = arg;
      Preferences prefs;
      if (prefs.begin("board", false)) {
        prefs.putString("name", arg);
        prefs.end();
      }
      enqueueMessage("Kart ismi ayarlandi: " + arg + "\nMAC: " + g_boardMac);
    }
    return;
  }

  if (norm == "yardim" || norm == "komutlar" || norm == "help" ||
      norm == "/help" || norm == "/start" || norm == "start") {
    enqueueMessage(
      displayName() + "\n\n"
      "Komutlar (Turkce):\n"
      "yardim - Bu mesaj\n" +
      displayName() + " durum - Bu kartin durumu\n"
      "hepsi durum - Tum kartlarin durumu\n" +
      displayName() + " bilgi - Cihaz bilgileri\n" +
      displayName() + " guncelle - Guncelleme baslat\n" +
      displayName() + " geri al - Onceki firmware\n" +
      displayName() + " yeniden baslat - Restart\n" +
      displayName() + " log - Son kayitlar\n" +
      displayName() + " isim <yeni isim> - Isim degistir");
  } else if (norm == "durum" || norm == "/status") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "State: " + pilotState + "\n";
    msg += "WiFi: " + WiFi.SSID() + "\n";
    msg += "Sinyal: " + String(WiFi.RSSI()) + " dBm\n";
    msg += "IP: " + WiFi.localIP().toString() + "\n";
    msg += "Uptime: " + formatUptime();
    enqueueMessage(msg);
  } else if (norm == "bilgi" || norm == "/info") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "FW: " + String(CURRENT_VERSION) + "\n";
    msg += "Partition: " + String(OTA_Manager::runningPartitionLabel()) + "\n";
    msg += "Image: " + String(OTA_Manager::runningImageStateLabel()) + "\n";
    msg += "Remote: " + String(OTA_Manager::lastRemoteVersion());
    enqueueMessage(msg);
  } else if (norm == "log" || norm == "kayit" || norm == "loglar" ||
             norm == "/log" || norm == "/logs") {
    char dump[360];
    app_log_dump(dump, sizeof(dump), 8);
    String msg = displayName() + " log\n";
    msg += (dump[0] ? dump : "(log bos)\n");
    enqueueMessage(msg);
  } else if (norm == "guncelle" || norm == "guncelleme" || norm == "/update") {
    String remote = OTA_Manager::lastRemoteVersion();
    String msg = "Guncelleme baslatildi...\n";
    msg += "Kart: " + displayName() + "\n";
    msg += "Mevcut: " + String(OTA_Manager::currentVersion());
    if (remote.length() > 0) msg += "\nHedef: " + remote;
    enqueueMessage(msg);
    OTA_Manager::triggerCheckNow();
    OTA_Manager::triggerInstallNow();
  } else if (norm == "geri al" || norm == "gerial" || norm == "/rollback") {
    enqueueMessage(displayName() + " - Onceki firmware'e donuluyor...");
    s_pendingAction = kActionRollback;
  } else if (norm == "yeniden baslat" || norm == "restart" || norm == "reboot" ||
             norm == "/restart") {
    enqueueMessage(displayName() + " - Yeniden baslatiliyor...");
    s_pendingAction = kActionRestart;
  } else {
    enqueueMessage("Anlasilmadi: " + rawCmd + "\n\nOrnek komutlar:\n" +
                   displayName() + " durum\nhepsi durum\nyardim");
  }
}

static void executePendingAction() {
  uint8_t action = s_pendingAction;
  if (action == kActionNone) return;
  s_pendingAction = kActionNone;

  if (action == kActionRestart) {
    delay(400);
    esp_restart();
  }

  if (action == kActionRollback) {
    delay(400);
    if (OTA_Manager::selectAlternateOtaBootPartition() ||
        OTA_Manager::selectFactoryBootPartition()) {
      delay(50);
      esp_restart();
    }
    enqueueMessage("Geri donulecek partition bulunamadi");
  }
}

static void pollIncoming() {
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;
  if (s_cmdReady) return;

  String url = "https://api.telegram.org/bot";
  url += TELEGRAM_BOT_TOKEN;
  url += "/getUpdates?offset=";
  url += String(s_lastUpdateId + 1);
  url += "&limit=1&timeout=0";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(1500);
  if (!http.begin(client, url)) return;

  int code = http.GET();
  if (code != 200) {
    http.end();
    return;
  }

  String body = http.getString();
  http.end();

  if (body.length() > 2048) return;
  if (body.indexOf("\"result\":[]") >= 0) return;

  int uidIdx = body.indexOf("\"update_id\":");
  if (uidIdx < 0) return;
  long uid = atol(body.c_str() + uidIdx + 12);
  if (uid > 0) {
    s_lastUpdateId = uid;
    s_uidDirty = true;
  }

  const char* chatPattern = "\"chat\":{\"id\":";
  int chatIdx = body.indexOf(chatPattern);
  if (chatIdx >= 0) {
    int idStart = chatIdx + (int)strlen(chatPattern);
    int idEnd = body.indexOf(',', idStart);
    if (idEnd < 0) idEnd = body.indexOf('}', idStart);
    String chatId = body.substring(idStart, idEnd);
    if (chatId != String(TELEGRAM_CHAT_ID)) return;
  }

  int textIdx = body.indexOf("\"text\":\"");
  if (textIdx < 0) return;
  int textStart = textIdx + 8;
  int textEnd = body.indexOf('"', textStart);
  if (textEnd < 0) return;
  String cmd = body.substring(textStart, textEnd);
  if (cmd.length() >= sizeof(s_pendingCmd)) {
    cmd = cmd.substring(0, sizeof(s_pendingCmd) - 1);
  }

  Serial.printf("[TG] Komut alindi: %s\n", cmd.c_str());
  portENTER_CRITICAL(&s_tgMux);
  strncpy(s_pendingCmd, cmd.c_str(), sizeof(s_pendingCmd) - 1);
  s_pendingCmd[sizeof(s_pendingCmd) - 1] = '\0';
  s_cmdReady = true;
  portEXIT_CRITICAL(&s_tgMux);
}

static void telegram_task(void* /*param*/) {
  uint32_t lastPollMs = 0;
  char msg[kOutMsgMax];
  for (;;) {
    const bool wifiOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
    bool didWork = false;

    if (wifiOk && dequeueMessage(msg, sizeof(msg))) {
      sendTelegramMessage(msg);
      didWork = true;
    } else if (outQueueIsEmpty()) {
      executePendingAction();
    }

    if (wifiOk && (millis() - lastPollMs) >= 3000) {
      lastPollMs = millis();
      pollIncoming();
      didWork = true;
    }

    vTaskDelay(pdMS_TO_TICKS(didWork ? 50 : 250));
  }
}

static void notifyConnect(const String& pilotState) {
  if (s_notified) return;
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;

  String msg = displayName();
  msg += "\n";
  msg += "WiFi: " + WiFi.SSID() + "\n";
  msg += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  msg += "IP: " + WiFi.localIP().toString() + "\n";
  msg += "FW: " + String(CURRENT_VERSION) + "\n";
  msg += "State: " + pilotState;
  enqueueMessage(msg);
  s_notified = true;
}

static void notifyWifiLink(bool connected) {
  if (connected) {
    if (!s_wifiWasConnected) {
      s_wifiWasConnected = true;
      if (s_wifiDownSinceMs != 0) {
        uint32_t downMs = millis() - s_wifiDownSinceMs;
        s_wifiDownSinceMs = 0;
        if (downMs >= 10000) {
          uint32_t sec = downMs / 1000;
          uint32_t m = sec / 60;
          uint32_t s = sec % 60;
          String msg = displayName() + "\n";
          msg += "Baglanti kesintisi yasandi\n";
          msg += "Kesinti suresi: ";
          if (m > 0) msg += String(m) + "dk ";
          msg += String(s) + "sn\n";
          msg += "Simdi tekrar bagliyim.";
          enqueueMessage(msg);
          Serial.printf("[TG] WiFi kesinti raporu: %lu sn\n", (unsigned long)sec);
        }
      }
    }
  } else if (s_wifiWasConnected) {
    s_wifiWasConnected = false;
    s_wifiDownSinceMs = millis();
    Serial.println("[TG] WiFi baglantisi koptu");
  }
}

static void notifyStateChange(const String& pilotState) {
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;

  if (s_lastNotifiedState[0] == '\0') {
    strncpy(s_lastNotifiedState, pilotState.c_str(), sizeof(s_lastNotifiedState) - 1);
    s_lastNotifiedState[sizeof(s_lastNotifiedState) - 1] = '\0';
    return;
  }
  if (pilotState == s_lastNotifiedState) return;

  String prevName, prevDesc, newName, newDesc;
  stateLabel(s_lastNotifiedState, prevName, prevDesc);
  stateLabel(pilotState.c_str(), newName, newDesc);

  String prevStateCode = s_lastNotifiedState;
  String msg = displayName() + "\n";
  msg += "State: " + prevName + " (" + prevStateCode + ")";
  msg += " -> ";
  msg += newName + " (" + pilotState + ")\n";
  msg += newDesc;

  strncpy(s_lastNotifiedState, pilotState.c_str(), sizeof(s_lastNotifiedState) - 1);
  s_lastNotifiedState[sizeof(s_lastNotifiedState) - 1] = '\0';
  enqueueMessage(msg);
  Serial.printf("[TG] State bildirimi: %s -> %s\n",
                prevStateCode.c_str(), pilotState.c_str());
}

void telegram_loop(const String& pilotState, bool wifiConnected) {
  if (!s_uidLoaded) {
    loadUpdateId();
    s_uidLoaded = true;
  }

  if (s_uidDirty) {
    s_uidDirty = false;
    saveUpdateId(s_lastUpdateId);
  }

  notifyWifiLink(wifiConnected);
  if (wifiConnected) {
    notifyConnect(pilotState);
    notifyStateChange(pilotState);
  }

  if (s_cmdReady) {
    char cmdCopy[96];
    portENTER_CRITICAL(&s_tgMux);
    strncpy(cmdCopy, s_pendingCmd, sizeof(cmdCopy) - 1);
    cmdCopy[sizeof(cmdCopy) - 1] = '\0';
    s_pendingCmd[0] = '\0';
    s_cmdReady = false;
    portEXIT_CRITICAL(&s_tgMux);
    handleCommand(String(cmdCopy), pilotState);
  }

  if (s_tgTaskHandle == NULL && strlen(TELEGRAM_BOT_TOKEN) > 0) {
    xTaskCreate(telegram_task, "tgbot", 16384, NULL, 1, &s_tgTaskHandle);
  }
}
