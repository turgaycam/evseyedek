#include "telegram_notify.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_system.h>
#include <Preferences.h>

#include "app_config.h"
#include "OTA_Manager.h"

#ifndef TELEGRAM_BOT_TOKEN
#define TELEGRAM_BOT_TOKEN ""
#endif

#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID ""
#endif

static bool s_notified = false;
static long s_lastUpdateId = 0;
static uint32_t s_lastPollMs = 0;
static const uint32_t kPollIntervalMs = 3000;
static bool s_uidLoaded = false;

// --- update_id'yi NVS'ye kaydet (reboot'ta komut tekrar islenmesin) ---
static void saveUpdateId(long uid) {
  Preferences prefs;
  if (prefs.begin("tgbot", false)) {
    prefs.putLong("last_uid", uid);
    prefs.end();
  }
}

// --- update_id'yi NVS'den yukle (ilk cagrida) ---
static void loadUpdateId() {
  Preferences prefs;
  if (prefs.begin("tgbot", true)) {
    s_lastUpdateId = prefs.getLong("last_uid", 0);
    prefs.end();
  }
  Serial.printf("[TG] Son update_id yuklendi: %ld\n", s_lastUpdateId);
}

// --- Gosterim ismi: ozel isim varsa onu, yoksa default kullan ---
static String displayName() {
  if (g_boardCustomName.length() > 0) return g_boardCustomName;
  return g_boardId + " (" + g_boardName + ")";
}

// --- URL encoder (mesaj icin, buyuk buffer yok) ---
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

// --- Telegram'a mesaj gonder ---
static void sendTelegramMessage(const String& text) {
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;

  String url = "https://api.telegram.org/bot";
  url += TELEGRAM_BOT_TOKEN;
  url += "/sendMessage?chat_id=";
  url += TELEGRAM_CHAT_ID;
  url += "&text=";
  url += urlEncode(text);

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

// --- Uptime formatla ---
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

// --- Cihaz baglaninca bildirim ---
void telegram_notify_connect(const String& pilotState) {
  // Ilk cagrida update_id'yi NVS'den yukle
  if (!s_uidLoaded) {
    s_uidLoaded = true;
    loadUpdateId();
  }

  if (s_notified) return;
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;

  s_notified = true;

  String msg = displayName();
  msg += " baglandi\n";
  msg += "MAC: " + g_boardMac + "\n";
  msg += "WiFi: " + WiFi.SSID() + "\n";
  msg += "Sinyal: " + String(WiFi.RSSI()) + " dBm\n";
  msg += "IP: " + WiFi.localIP().toString() + "\n";
  msg += "FW: " + String(CURRENT_VERSION) + "\n";
  msg += "State: " + pilotState + "\n";
  msg += "\nKomutlar icin /help yaz";

  sendTelegramMessage(msg);
}

// --- Komut isle ---
static void handleCommand(const String& cmd, const String& pilotState) {
  // /name komutu: /name Garaj1 -> ismi ayarla, /name -> mevcut ismi goster
  if (cmd.startsWith("/name")) {
    String arg = cmd.substring(5);
    arg.trim();
    if (arg.length() == 0) {
      String msg = "Kart ismi: " + displayName() + "\n";
      msg += "MAC: " + g_boardMac + "\n";
      msg += "Degistirmek icin: /name YeniIsim";
      sendTelegramMessage(msg);
    } else if (arg.length() > 30) {
      sendTelegramMessage("Isim cok uzun (max 30 karakter)");
    } else {
      g_boardCustomName = arg;
      Preferences prefs;
      if (prefs.begin("board", false)) {
        prefs.putString("name", arg);
        prefs.end();
      }
      sendTelegramMessage("Kart ismi ayarlandi: " + arg + "\nMAC: " + g_boardMac);
    }
    return;
  }

  if (cmd == "/help" || cmd == "/start") {
    sendTelegramMessage(
      String(displayName() + "\n\n"
      "Komutlar:\n"
      "/help - Bu mesaj\n"
      "/status - Cihaz durumu (WiFi, state, uptime)\n"
      "/info - Cihaz bilgileri (MAC, FW, partition)\n"
      "/name - Kart ismini goster\n"
      "/name <isim> - Kart ismini ayarla (orn: /name Garaj1)\n"
      "/update - Son surumu kontrol et + yukle\n"
      "/rollback - Onceki firmware'e don\n"
      "/restart - Cihazi yeniden baslat")
    );
  }
  else if (cmd == "/status") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "State: " + pilotState + "\n";
    msg += "WiFi: " + WiFi.SSID() + "\n";
    msg += "Sinyal: " + String(WiFi.RSSI()) + " dBm\n";
    msg += "IP: " + WiFi.localIP().toString() + "\n";
    msg += "Uptime: " + formatUptime();
    sendTelegramMessage(msg);
  }
  else if (cmd == "/info") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "FW: " + String(CURRENT_VERSION) + "\n";
    msg += "Partition: " + String(OTA_Manager::runningPartitionLabel()) + "\n";
    msg += "Image: " + String(OTA_Manager::runningImageStateLabel()) + "\n";
    msg += "Remote: " + String(OTA_Manager::lastRemoteVersion());
    sendTelegramMessage(msg);
  }
  else if (cmd == "/update") {
    String remote = OTA_Manager::lastRemoteVersion();
    String msg = "Guncelleme baslatildi...\n";
    msg += "Kart: " + displayName() + "\n";
    msg += "Mevcut: " + String(OTA_Manager::currentVersion());
    if (remote.length() > 0) msg += "\nHedef: " + remote;
    sendTelegramMessage(msg);
    OTA_Manager::triggerCheckNow();
    OTA_Manager::triggerInstallNow();
  }
  else if (cmd == "/rollback") {
    sendTelegramMessage(displayName() + " - Onceki firmware'e donuluyor...");
    delay(1000);
    if (OTA_Manager::selectAlternateOtaBootPartition()) {
      delay(100);
      esp_restart();
    } else if (OTA_Manager::selectFactoryBootPartition()) {
      delay(100);
      esp_restart();
    } else {
      sendTelegramMessage("Geri donulecek partition bulunamadi");
    }
  }
  else if (cmd == "/restart") {
    sendTelegramMessage(displayName() + " - Yeniden baslatiliyor...");
    delay(1000);
    esp_restart();
  }
  else {
    sendTelegramMessage("Bilinmeyen komut: " + cmd + "\n/help yazarak komutlari gorebilirsin");
  }
}

// --- Telegram'dan komut dinle (main loop'ta cagir) ---
void telegram_loop(const String& pilotState) {
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;

  uint32_t now = millis();
  if (now - s_lastPollMs < kPollIntervalMs) return;
  s_lastPollMs = now;

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

  // Response cok buyukse atla (memory korumasi)
  if (body.length() > 2048) {
    Serial.printf("[TG] Response cok buyuk: %u byte, ataniyor\n", body.length());
    return;
  }

  // Bos result kontrolu
  if (body.indexOf("\"result\":[]") >= 0) return;

  // update_id ayikla
  int uidIdx = body.indexOf("\"update_id\":");
  if (uidIdx < 0) return;
  long uid = atol(body.c_str() + uidIdx + 12);
  if (uid > 0) {
    s_lastUpdateId = uid;
    saveUpdateId(uid);  // NVS'ye kaydet - reboot'ta tekrar islenmesin!
  }

  // chat ID dogrula (guvenlik)
  const char* chatPattern = "\"chat\":{\"id\":";
  int chatIdx = body.indexOf(chatPattern);
  if (chatIdx >= 0) {
    int idStart = chatIdx + strlen(chatPattern);
    int idEnd = body.indexOf(',', idStart);
    if (idEnd < 0) idEnd = body.indexOf('}', idStart);
    String chatId = body.substring(idStart, idEnd);
    if (chatId != String(TELEGRAM_CHAT_ID)) return;  // yetkisiz kullanici
  }

  // text ayikla
  int textIdx = body.indexOf("\"text\":\"");
  if (textIdx < 0) return;
  int textStart = textIdx + 8;
  int textEnd = body.indexOf('"', textStart);
  if (textEnd < 0) return;
  String cmd = body.substring(textStart, textEnd);

  // Komutu isle
  Serial.printf("[TG] Komut alindi: %s\n", cmd.c_str());
  handleCommand(cmd, pilotState);
}