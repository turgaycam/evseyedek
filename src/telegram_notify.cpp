#include "telegram_notify.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "app_config.h"
#include "OTA_Manager.h"

#ifndef TELEGRAM_BOT_TOKEN
#define TELEGRAM_BOT_TOKEN ""
#endif

#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID ""
#endif

static bool s_notified = false;

// Minimal URL encoder — sadece mesaj icin, buyuk buffer yok.
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

void telegram_notify_connect(const String& pilotState) {
  if (s_notified) return;
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;

  s_notified = true;  // Tek seferlik gonderim — tekrar baglansa da spam yapmaz

  String msg = g_boardId;
  msg += " (";
  msg += g_boardName;
  msg += ") baglandi\n";
  msg += "WiFi: ";
  msg += WiFi.SSID();
  msg += "\nSinyal: ";
  msg += String(WiFi.RSSI());
  msg += " dBm\nIP: ";
  msg += WiFi.localIP().toString();
  msg += "\nFW: ";
  msg += CURRENT_VERSION;
  msg += "\nState: ";
  msg += pilotState;

  String url = "https://api.telegram.org/bot";
  url += TELEGRAM_BOT_TOKEN;
  url += "/sendMessage?chat_id=";
  url += TELEGRAM_CHAT_ID;
  url += "&text=";
  url += urlEncode(msg);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(5000);
  if (http.begin(client, url)) {
    int code = http.GET();
    if (code != 200) {
      Serial.printf("[TG] Telegram gonderim hatasi: HTTP %d\n", code);
    } else {
      Serial.println("[TG] Telegram mesaji gonderildi");
    }
    http.end();
  }
}