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

#ifndef TELEGRAM_BOT_TOKEN
#define TELEGRAM_BOT_TOKEN ""
#endif

#ifndef TELEGRAM_CHAT_ID
#define TELEGRAM_CHAT_ID ""
#endif

static bool s_notified = false;
static long s_lastUpdateId = 0;
static String s_lastNotifiedState = "";
static bool s_uidLoaded = false;
static TaskHandle_t s_tgTaskHandle = NULL;

static String s_pendingCmd;
static volatile bool s_cmdReady = false;

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

// IEC 61851 state etiketleri (web arayuzuyle uyumlu).
static void stateLabel(const String& st, String& name, String& desc) {
  if (st == "A") { name = "Hazır"; desc = "Araç bekleniyor"; }
  else if (st == "B") { name = "Bağlı"; desc = "Araç bağlandı, hazır bekliyor"; }
  else if (st == "C") { name = "Şarj Ediliyor"; desc = "Enerji aktarımı sürüyor ⚡"; }
  else if (st == "D") { name = "Havalandırma"; desc = "Şarj sürüyor (fan/havalandırma)"; }
  else if (st == "E") { name = "Şarj Hatası"; desc = "Pilot hata durumu ⚠️"; }
  else if (st == "F") { name = "Kritik Hata"; desc = "Koruma aktif, şarj durduruldu 🛑"; }
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

// Kendi istasyon kodunu MAC'ten turet (son 6 hex karakter).
// Ornek MAC "3C:DC:75:55:3B:48" -> "553B48"
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

// Turkce karakterleri ASCII karsiliklarina cevirir + lowercase yapar.
// Karsilastirmalarda kullanilir ("Pangolin" == "pangolin", "DURUM" == "durum").
static String normalizeTr(const String& in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length();) {
    uint8_t c = (uint8_t)in[i];
    if (c == 0xC3 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0xA7 || c2 == 0x87) { out += 'c'; i += 2; continue; } // c/C
      if (c2 == 0xB6 || c2 == 0x96) { out += 'o'; i += 2; continue; } // o/O
      if (c2 == 0xBC || c2 == 0x9C) { out += 'u'; i += 2; continue; } // u/U
    } else if (c == 0xC4 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0x9F || c2 == 0x9E) { out += 'g'; i += 2; continue; } // g/G
      if (c2 == 0xB1 || c2 == 0xB0) { out += 'i'; i += 2; continue; } // i/I
    } else if (c == 0xC5 && i + 1 < in.length()) {
      uint8_t c2 = (uint8_t)in[i + 1];
      if (c2 == 0x9F || c2 == 0x9E) { out += 's'; i += 2; continue; } // s/S
    }
    char ascii = (char)c;
    if (ascii >= 'A' && ascii <= 'Z') ascii = ascii - 'A' + 'a';
    out += ascii;
    i++;
  }
  return out;
}

// Mesajin basi bu kartin adiyla eslesiyor mu? (cok kelimeli ve bitisik yazim dahil)
// Ornekler: kart adi "Ters Lale" ise su mesajlar eslesir:
//   "ters lale durum"  -> bosluklu yazim
//   "terslale durum"   -> bitisik yazim
// Donen deger: eslesen ismin normalize edilmis uzunlugu (0 = eslesme yok).
static int matchMyNamePrefix(const String& normMsg) {
  String base[3];
  base[0] = normalizeTr(displayName());
  base[1] = normalizeTr(g_boardCustomName);
  base[2] = normalizeTr(g_boardName);

  // Hem normal hem bosluksuz versiyonlari dene ("ters lale" + "terslale").
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

  // Turkce komut destegi: mesaj normalize edilir (turkce karakterler -> ascii,
  // lowercase). Ornekler:
  //   "pangolin durum"        -> yalnizca Pangolin durumu bildirir
  //   "hepsi durum"           -> tum kartlar bildirir
  //   "kakapo guncelle"       -> Kakapo guncelleme baslatir
  //   "yardim"                -> komut listesi
  String norm = normalizeTr(cmd);

  String target = "";
  if (norm.startsWith("/")) {
    // Slash komutlari: "/<stationCode> <komut>" veya "/all <komut>"
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
    // Turkce komutlar: ilk kelime hedef olabilir (kart adi / hepsi / kod).
    int sp = norm.indexOf(' ');
    String firstTok = (sp > 0) ? norm.substring(0, sp) : "";
    String rest = (sp > 0) ? norm.substring(sp + 1) : "";
    rest.trim();

    // Cok kelimeli isim destegi: mesaj basi kart adiyla eslesiyor mu?
    // Ornek: "mersin tatlısı durum" -> hedef=Mersin Tatlısı, komut="durum"
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
      // Ilk kelime komut (hedefsiz): tum mesaj komut sayilir.
      cmd = norm;
    }
  }

  const String myCode = myStationCode();
  if (target.length() > 0 && target != "ALL" && target != myCode) {
    Serial.printf("[TG] Komut baska karta ait (%s), yok sayildi\n", target.c_str());
    return;
  }

  // Aksiyon komutlarinda hedef zorunlu; aksi halde ayni bota bagli
  // tum kartlar ayni anda cevap verip karisiklik yaratir.
  bool needsTarget =
    norm.startsWith("durum") || norm.startsWith("bilgi") ||
    norm.startsWith("guncelle") || norm.startsWith("geri al") ||
    norm.startsWith("gerial") || norm.startsWith("yeniden baslat") ||
    norm.startsWith("isim") ||
    norm.startsWith("/status") || norm.startsWith("/info") ||
    norm.startsWith("/update") || norm.startsWith("/rollback") ||
    norm.startsWith("/restart") || norm.startsWith("/name");
  if (needsTarget && target.length() == 0) {
    sendTelegramMessage(
      String(displayName() + "\n"
      "Birden fazla istasyon ayni bota bagli.\n"
      "Komutu kart adiyla gonderin, orn:\n"
      "" + displayName() + " durum\n"
      "Tum kartlara: hepsi durum")
    );
    return;
  }

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

  if (norm == "yardim" || norm == "komutlar" || norm == "help" ||
      norm == "/help" || norm == "/start" || norm == "start") {
    sendTelegramMessage(
      String(displayName() + "\n\n"
      "Komutlar (Turkce):\n"
      "yardim - Bu mesaj\n"
      "" + displayName() + " durum - Bu kartin durumu\n"
      "hepsi durum - Tum kartlarin durumu\n"
      "" + displayName() + " bilgi - Cihaz bilgileri\n"
      "" + displayName() + " guncelle - Guncelleme baslat\n"
      "" + displayName() + " geri al - Onceki firmware\n"
      "" + displayName() + " yeniden baslat - Restart\n"
      "" + displayName() + " isim <yeni isim> - Isim degistir")
    );
  }
  else if (norm == "durum" || norm == "/status") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "State: " + pilotState + "\n";
    msg += "WiFi: " + WiFi.SSID() + "\n";
    msg += "Sinyal: " + String(WiFi.RSSI()) + " dBm\n";
    msg += "IP: " + WiFi.localIP().toString() + "\n";
    msg += "Uptime: " + formatUptime();
    sendTelegramMessage(msg);
  }
  else if (norm == "bilgi" || norm == "/info") {
    String msg = displayName() + "\n";
    msg += "MAC: " + g_boardMac + "\n";
    msg += "FW: " + String(CURRENT_VERSION) + "\n";
    msg += "Partition: " + String(OTA_Manager::runningPartitionLabel()) + "\n";
    msg += "Image: " + String(OTA_Manager::runningImageStateLabel()) + "\n";
    msg += "Remote: " + String(OTA_Manager::lastRemoteVersion());
    sendTelegramMessage(msg);
  }
  else if (norm == "guncelle" || norm == "guncelleme" || norm == "/update") {
    String remote = OTA_Manager::lastRemoteVersion();
    String msg = "Guncelleme baslatildi...\n";
    msg += "Kart: " + displayName() + "\n";
    msg += "Mevcut: " + String(OTA_Manager::currentVersion());
    if (remote.length() > 0) msg += "\nHedef: " + remote;
    sendTelegramMessage(msg);
    OTA_Manager::triggerCheckNow();
    OTA_Manager::triggerInstallNow();
  }
  else if (norm == "geri al" || norm == "gerial" || norm == "/rollback") {
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
  else if (norm == "yeniden baslat" || norm == "restart" || norm == "reboot" || norm == "/restart") {
    sendTelegramMessage(displayName() + " - Yeniden baslatiliyor...");
    delay(1000);
    esp_restart();
  }
  else {
    sendTelegramMessage(
      String("Anlasilmadi: " + rawCmd + "\n\nOrnek komutlar:\n"
      "" + displayName() + " durum\nhepsi durum\nyardim")
    );
  }
}

static void telegram_task(void* param) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(3000));

    if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) continue;
    if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) continue;
    if (s_cmdReady) continue;

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
    if (!http.begin(client, url)) continue;

    int code = http.GET();
    if (code != 200) {
      http.end();
      continue;
    }

    String body = http.getString();
    http.end();

    if (body.length() > 2048) continue;
    if (body.indexOf("\"result\":[]") >= 0) continue;

    int uidIdx = body.indexOf("\"update_id\":");
    if (uidIdx < 0) continue;
    long uid = atol(body.c_str() + uidIdx + 12);
    if (uid > 0) {
      s_lastUpdateId = uid;
      saveUpdateId(uid);
    }

    const char* chatPattern = "\"chat\":{\"id\":";
    int chatIdx = body.indexOf(chatPattern);
    if (chatIdx >= 0) {
      int idStart = chatIdx + strlen(chatPattern);
      int idEnd = body.indexOf(',', idStart);
      if (idEnd < 0) idEnd = body.indexOf('}', idStart);
      String chatId = body.substring(idStart, idEnd);
      if (chatId != String(TELEGRAM_CHAT_ID)) continue;
    }

    int textIdx = body.indexOf("\"text\":\"");
    if (textIdx < 0) continue;
    int textStart = textIdx + 8;
    int textEnd = body.indexOf('"', textStart);
    if (textEnd < 0) continue;
    String cmd = body.substring(textStart, textEnd);

    Serial.printf("[TG] Komut alindi: %s\n", cmd.c_str());

    s_pendingCmd = cmd;
    s_cmdReady = true;
  }
}

void telegram_notify_connect(const String& pilotState) {
  if (s_notified) return;
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;

  String msg = "";
  msg += displayName();
  msg += "\n";
  msg += "WiFi: " + WiFi.SSID() + "\n";
  msg += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  msg += "IP: " + WiFi.localIP().toString() + "\n";
  msg += "FW: " + String(CURRENT_VERSION) + "\n";
  msg += "State: " + pilotState;
  sendTelegramMessage(msg);
  s_notified = true;
}

// WiFi baglanti kesintisi takibi.
// Baglanti koptugunda sure sayilir; geri gelindiginde kesinti raporu gonderilir.
// Not: Baglanti kopukken internet erisimi olmadigindan mesaj ancak reconnect
// sonrasi gonderilebilir. 10 sn'den kisa dalgalanmalar bildirilmez (spam onleme).
static bool s_wifiWasConnected = false;
static uint32_t s_wifiDownSinceMs = 0;

void telegram_notify_wifi_link(bool connected) {
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
          msg += "⚠️ Bağlantı kesintisi yaşadım\n";
          msg += "Kesinti süresi: ";
          if (m > 0) msg += String(m) + "dk ";
          msg += String(s) + "sn\n";
          msg += "Şimdi tekrar bağlıyım.";
          sendTelegramMessage(msg);
          Serial.printf("[TG] WiFi kesinti raporu: %lu sn\n", (unsigned long)sec);
        }
      }
    }
  } else {
    if (s_wifiWasConnected) {
      s_wifiWasConnected = false;
      s_wifiDownSinceMs = millis();
      Serial.println("[TG] WiFi baglantisi koptu");
    }
  }
}

// Pilot state degistiginde Telegram'a bildirim gonderir.
// Ilk cagrista sadece referans state kaydedilir (boot bildirimi ayrica yapilir).
void telegram_notify_state_change(const String& pilotState) {
  if (strlen(TELEGRAM_BOT_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return;
  if (WiFi.status() != WL_CONNECTED || WiFi.localIP()[0] == 0) return;

  if (s_lastNotifiedState.length() == 0) {
    s_lastNotifiedState = pilotState;
    return;
  }
  if (pilotState == s_lastNotifiedState) return;

  String prevName, prevDesc, newName, newDesc;
  stateLabel(s_lastNotifiedState, prevName, prevDesc);
  stateLabel(pilotState, newName, newDesc);

  String prevStateCode = s_lastNotifiedState;
  String msg = displayName() + "\n";
  msg += "State: " + prevName + " (" + prevStateCode + ")";
  msg += " -> ";
  msg += newName + " (" + pilotState + ")\n";
  msg += newDesc;

  s_lastNotifiedState = pilotState;
  sendTelegramMessage(msg);
  Serial.printf("[TG] State bildirimi: %s -> %s\n",
                prevStateCode.c_str(), pilotState.c_str());
}

void telegram_loop(const String& pilotState) {
  if (!s_uidLoaded) {
    loadUpdateId();
    s_uidLoaded = true;
  }

  if (s_cmdReady) {
    handleCommand(s_pendingCmd, pilotState);
    s_cmdReady = false;
    s_pendingCmd = "";
  }

  if (s_tgTaskHandle == NULL && strlen(TELEGRAM_BOT_TOKEN) > 0) {
    xTaskCreate(telegram_task, "tgbot", 16384, NULL, 1, &s_tgTaskHandle);
  }
}