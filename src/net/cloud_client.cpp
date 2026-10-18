#include "cloud_client.h"

#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <string.h>
#include <time.h>

#include "OTA_Manager.h"
#include "cloud_certs.h"
#include "cloud_core.h"
#include "app_pins.h"
#include "io/relay.h"
#include "schedule.h"

using namespace evse_cloud;

extern int g_chargeMode;
extern uint32_t g_manualStopAlertUntilMs;
extern uint32_t g_manualStopAutoResumeAtMs;

namespace {

constexpr const char* kPrefsNs = "evsecloud";
constexpr uint32_t kOnlineWindowMs = 15000;   // sunucunun "cevrimici" esigiyle ayni

// NVS'deki ayar (anahtar dahil). Yalnizca bu dosyada; disari asla kopyalanmaz.
// on varsayilan kapali, url varsayilan bos: kart hicbir buluta baglanmaz.
struct Config {
  bool on = false;
  char code[17] = "";            // bos = varsayilan (MAC son 6 hane)
  char url[kUrlMax] = "";        // bos = bulut yok
  char secret[kSecretLen + 1] = "";
};

portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

Config s_cfg;                    // s_mux ile korunur
uint32_t s_cfgGen = 0;           // ayar degisince artar (gorev baglantiyi yeniler)
volatile bool s_requireStart = false;
char s_macCode[7] = "";

// Ana dongu -> gorev
Telemetry s_tel;
bool s_telValid = false;

// Gorev <-> ana dongu komut yuvasi
bool s_cmdPending = false;
bool s_cmdDone = false;
int64_t s_cmdId = 0;
CmdType s_cmdType = CmdType::None;
bool s_cmdOk = false;

// Gorev -> herkes (durum)
bool s_everOk = false;
uint32_t s_lastOkMs = 0;
int s_lastHttp = 0;
uint32_t s_failures = 0;
const char* s_phase = "ayar_yok";
int64_t s_lastCmdId = 0;
bool s_lastCmdOk = false;
CmdType s_lastCmdType = CmdType::None;

// Yalnizca ana dongu
SafeGate s_gate;
bool s_prevPermit = true;   // ilk turda izin yoksa da role guvenle birakilsin
bool s_softStop = false;
uint32_t s_softStopAtMs = 0;
bool s_cloudStopLatched = false;
volatile bool s_paused = false;

void setPhase(const char* p) {
  portENTER_CRITICAL(&s_mux);
  s_phase = p;
  portEXIT_CRITICAL(&s_mux);
}

void ensureMacCode() {
  if (s_macCode[0]) return;
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  snprintf(s_macCode, sizeof(s_macCode), "%02X%02X%02X", mac[3], mac[4], mac[5]);
}

// Kilit altinda cagrilir.
const char* effCode(const Config& c) { return c.code[0] ? c.code : s_macCode; }
const char* effUrl(const Config& c) { return c.url; }
bool isActive(const Config& c) {
  return c.on && c.url[0] != '\0' && valid_secret(c.secret) && valid_station_code(effCode(c));
}

bool activeNow() {
  portENTER_CRITICAL(&s_mux);
  bool a = isActive(s_cfg);
  portEXIT_CRITICAL(&s_mux);
  return a;
}

// PWM bu turda kapatilir (cloud_loop false doner / STOP modu); role, arac C/D'den ciktiginda
// ya da en gec kSoftStopMaxWaitMs sonra mevcut guvenli kapatma yoluyla (relay_force_off_now) birakilir.
void beginSoftStop(uint32_t nowMs) {
  if (s_softStop) return;
  s_softStop = true;
  s_softStopAtMs = nowMs;
}

void serviceSoftStop(char st, uint32_t nowMs) {
  if (!s_softStop) return;
#if RELAY_USE_LATCH_PINS
  // Eski latch kart: PWM kapandiktan sonra role birakilir. Yeni kartta bu yol kapali.
  bool drawing = (st == 'C' || st == 'D');
  if (drawing && (uint32_t)(nowMs - s_softStopAtMs) < kSoftStopMaxWaitMs) return;
  relay_force_off_now();
  s_softStop = false;
  Serial.println("[GATE] Guvenli durdurma: role birakildi");
#else
  // Yeni kart: GPIO4 state C/D boyunca seviye HIGH. Buradan force_off cagirmak,
  // state hâlâ C/D ise kontaktoru bir tur birakip sonraki tur yeniden ceker.
  // Durdurma PWM ile yapilir; arac C/D'den cikinca relay_update_auto LOW yapar.
  (void)st;
  (void)nowMs;
  s_softStop = false;
#endif
}

// Komutu mevcut sarj kontrol yoluna cevirir (g_chargeMode + guvenli mod kapisi). Donus = ackOk.
bool applyCommand(CmdType type, char st, bool require, uint32_t nowMs) {
  switch (type) {
    case CmdType::Start: {
      // Zamanlayici yasakli araliktaysa da kabul edilir: izin saklanir, aralik bitince baslar.
      bool ok = s_gate.on_start(st, require);
      if (ok) {
        // Panel AUTO ile ayni: manuel/bulut STOP kilidini kaldir.
        if (g_chargeMode == 2) g_chargeMode = 0;
        g_manualStopAlertUntilMs = 0;
        g_manualStopAutoResumeAtMs = 0;
        s_cloudStopLatched = false;
      }
      return ok;
    }
    case CmdType::Stop: {
      s_gate.on_stop();
      // Panel STOP ile ayni mod (2), ancak 60 sn sonra kendiliginden AUTO'ya donmez:
      // arac ayrilinca (A) AUTO'ya doner. requireCloudStart acikken izin de kalkar.
      g_chargeMode = 2;
      g_manualStopAutoResumeAtMs = 0;
      s_cloudStopLatched = true;
      beginSoftStop(nowMs);
      return true;
    }
    default:
      return false;  // set_limit ve bilinmeyen komutlar desteklenmiyor
  }
}

void rmKey(Preferences& p, const char* key) {
  if (p.isKey(key)) p.remove(key);
}

void persistLast(int64_t id, bool ok) {
  Preferences p;
  if (!p.begin(kPrefsNs, false)) return;
  p.putLong64("lastId", id);
  p.putBool("lastOk", ok);
  p.putBool("hasLast", true);
  p.end();
}

CommandTracker s_tracker;  // yalnizca bulut gorevi

// Ana dongunun uyguladigi komutun sonucunu al (gorev baglami).
void collectAppliedResult() {
  bool done = false;
  int64_t id = 0;
  bool ok = false;
  CmdType type = CmdType::None;
  portENTER_CRITICAL(&s_mux);
  if (s_cmdPending && s_cmdDone) {
    done = true;
    id = s_cmdId;
    ok = s_cmdOk;
    type = s_cmdType;
    s_cmdPending = false;
    s_cmdDone = false;
    s_lastCmdId = id;
    s_lastCmdOk = ok;
    s_lastCmdType = type;
  }
  portEXIT_CRITICAL(&s_mux);
  if (!done) return;
  s_tracker.on_applied(id, ok);
  persistLast(id, ok);
  Serial.printf("[CLOUD] Komut id=%lld tip=%s uygulandi, ok=%d\n", (long long)id, cmd_type_name(type), ok ? 1 : 0);
}

void handleCommand(const PollResponse& r) {
  CommandTracker::Decision d = s_tracker.on_command(r.cmdId);
  if (d == CommandTracker::Decision::Apply) {
    portENTER_CRITICAL(&s_mux);
    s_cmdPending = true;
    s_cmdDone = false;
    s_cmdId = r.cmdId;
    s_cmdType = r.type;
    portEXIT_CRITICAL(&s_mux);
    Serial.printf("[CLOUD] Komut alindi id=%lld tip=%s\n", (long long)r.cmdId, cmd_type_name(r.type));
  } else if (d == CommandTracker::Decision::ReAck) {
    Serial.printf("[CLOUD] Komut id=%lld tekrar geldi: uygulanmadi, yeniden onaylanacak\n", (long long)r.cmdId);
  }
}

void cloudTask(void*) {
  static WiFiClientSecure client;
  static HTTPClient http;
  client.setCACert(kCloudRootCAs);              // dogrulama zorunlu; setInsecure YOK
  client.setHandshakeTimeout(6);                // sn
  http.setReuse(true);                           // keep-alive
  http.setConnectTimeout(4000);
  http.setTimeout(4000);

  static Config cfg;          // gorev kopyasi (anahtar dahil); her tur yenilenir
  static char url[kUrlMax + 24];
  static char body[512];
  uint32_t usedGen = 0xFFFFFFFFu;
  int lastLoggedHttp = 0;

  for (;;) {
    bool active;
    uint32_t gen;
    portENTER_CRITICAL(&s_mux);
    cfg = s_cfg;
    gen = s_cfgGen;
    active = isActive(cfg);
    if (!cfg.code[0]) memcpy(cfg.code, s_macCode, sizeof(s_macCode));
    portEXIT_CRITICAL(&s_mux);

    if (gen != usedGen) {
      // Ayar degisti: eski baglantiyi kapat, oturum durumunu sifirla.
      bool first = (usedGen == 0xFFFFFFFFu);
      usedGen = gen;
      if (client.connected()) client.stop();
      if (!first) {
        // Istasyon kodu degistiyse NVS'deki komut gecmisi silinmistir; RAM'deki takip de yenilenir.
        Preferences p;
        bool hasLast = false, lastOk = false;
        int64_t lastId = 0;
        if (p.begin(kPrefsNs, true)) {
          hasLast = p.getBool("hasLast", false);
          lastId = p.getLong64("lastId", 0);
          lastOk = p.getBool("lastOk", false);
          p.end();
        }
        s_tracker = CommandTracker();
        s_tracker.restore(hasLast, lastId, lastOk);
      }
      portENTER_CRITICAL(&s_mux);
      s_everOk = false;
      s_failures = 0;
      s_lastHttp = 0;
      portEXIT_CRITICAL(&s_mux);
      lastLoggedHttp = 0;
    }
    if (!active) {
      memset(cfg.secret, 0, sizeof(cfg.secret));
      setPhase(cfg.on ? "ayar_yok" : "kapali");
      if (client.connected()) client.stop();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    bool staOk = (WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0);
    if (!staOk) {
      setPhase("wifi_yok");
      if (client.connected()) client.stop();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    // NTP schedule.cpp tarafindan baslatilir; saat gelmeden TLS denenmez.
    if ((int64_t)time(nullptr) < evse_sched::kMinValidEpoch) {
      setPhase("saat_bekleniyor");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    collectAppliedResult();

    Telemetry t;
    bool telValid;
    portENTER_CRITICAL(&s_mux);
    t = s_tel;
    telValid = s_telValid;
    portEXIT_CRITICAL(&s_mux);
    if (!telValid) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    snprintf(url, sizeof(url), "%s/api/device/poll", effUrl(cfg));
    Ack ack = s_tracker.pending_ack();
    int len = build_poll_json(body, sizeof(body), cfg.code, cfg.secret, t, OTA_Manager::currentVersion(), ack);
    memset(cfg.secret, 0, sizeof(cfg.secret));
    if (len <= 0) {
      setPhase("hata");
      vTaskDelay(pdMS_TO_TICKS(kBackoffMaxMs));
      continue;
    }

    int code = -1;
    bool parsedOk = false;
    PollResponse resp;
    if (http.begin(client, url)) {
      http.addHeader("Content-Type", "application/json");
      code = http.POST(reinterpret_cast<uint8_t*>(body), (size_t)len);
      if (code > 0) {
        int size = http.getSize();
        if (size <= 1024) {
          String s = http.getString();   // her durumda oku: baglanti yeniden kullanilabilsin
          if (code == 200) parsedOk = parse_poll_response(s.c_str(), s.length(), resp) && resp.ok;
        }
      }
      http.end();
    }
    memset(body, 0, sizeof(body));  // gizli anahtar tamponda kalmasin
    if (code <= 0) client.stop();   // tasima hatasi: temiz kapat, sonraki turda yeniden baglan

    // Bu sirada ayar degistiyse yaniti yok say (eski istasyon/anahtar).
    bool stale;
    portENTER_CRITICAL(&s_mux);
    stale = (s_cfgGen != usedGen);
    portEXIT_CRITICAL(&s_mux);
    if (stale) continue;

    uint32_t nowMs = millis();
    if (parsedOk) {
      if (ack.has) s_tracker.on_ack_delivered(ack.id);
      // Yalnizca bu yanitta gelen (sunucuda hala gecerli) komut uygulanir.
      if (resp.hasCmd) handleCommand(resp);
    }
    portENTER_CRITICAL(&s_mux);
    s_lastHttp = code;
    if (parsedOk) {
      s_everOk = true;
      s_lastOkMs = nowMs;
      s_failures = 0;
      s_phase = "bagli";
    } else {
      if (s_failures < 1000) s_failures++;
      s_phase = (code == 401 || code == 403) ? "anahtar_reddedildi" : "hata";
    }
    uint32_t failures = s_failures;
    portEXIT_CRITICAL(&s_mux);

    if (code != lastLoggedHttp) {
      lastLoggedHttp = code;
      Serial.printf("[CLOUD] Yoklama HTTP %d%s\n", code, parsedOk ? "" : " (basarisiz)");
    }
    vTaskDelay(pdMS_TO_TICKS(backoff_delay_ms(failures)));
  }
}

}  // namespace

void cloud_init() {
  ensureMacCode();
  Config c;
  bool hasLast = false;
  int64_t lastId = 0;
  bool lastOk = false;
  bool req = false;
  Preferences p;
  if (p.begin(kPrefsNs, true)) {
    c.on = p.getBool("on", false);
    p.getString("code", c.code, sizeof(c.code));
    p.getString("url", c.url, sizeof(c.url));
    p.getString("secret", c.secret, sizeof(c.secret));
    req = p.getBool("reqStart", false);
    hasLast = p.getBool("hasLast", false);
    lastId = p.getLong64("lastId", 0);
    lastOk = p.getBool("lastOk", false);
    p.end();
  }
  // Bozuk kayitlar yok sayilir.
  if (c.code[0] && !valid_station_code(c.code)) c.code[0] = '\0';
  char tmp[kUrlMax];
  if (c.url[0] && !normalize_server_url(c.url, tmp)) c.url[0] = '\0';
  if (c.secret[0] && !valid_secret(c.secret)) memset(c.secret, 0, sizeof(c.secret));
  s_tracker.restore(hasLast, lastId, lastOk);
  s_requireStart = req;

  portENTER_CRITICAL(&s_mux);
  s_cfg = c;
  s_cfgGen++;
  s_lastCmdId = hasLast ? lastId : 0;
  s_lastCmdOk = lastOk;
  bool active = isActive(s_cfg);
  portEXIT_CRITICAL(&s_mux);
  memset(c.secret, 0, sizeof(c.secret));

  BaseType_t ok = xTaskCreatePinnedToCore(cloudTask, "cloud", 10240, nullptr, 1, nullptr, 0);
  Serial.printf("[CLOUD] Bulut gorevi %s; durum: %s (requireCloudStart=%d)\n",
                ok == pdPASS ? "basladi" : "BASLATILAMADI",
                active ? "etkin" : "Bulut ayari yapilmadi / kapali", req ? 1 : 0);
}

bool cloud_active() { return activeNow(); }
bool cloud_require_start() { return s_requireStart && activeNow(); }

bool cloud_apply_config(const CloudCfgChange& ch, const char** err) {
  const char* dummy;
  if (err == nullptr) err = &dummy;
  *err = "";
  ensureMacCode();

  Config next;
  portENTER_CRITICAL(&s_mux);
  next = s_cfg;
  portEXIT_CRITICAL(&s_mux);

  // 1) Dogrulama (hicbir sey yazilmadan)
  if (ch.code != nullptr) {
    if (ch.code[0] == '\0') {
      next.code[0] = '\0';
    } else if (!valid_station_code(ch.code)) {
      *err = "Istasyon kodu 3-16 karakter, yalnizca A-Z 0-9 _ - olmali";
      return false;
    } else {
      snprintf(next.code, sizeof(next.code), "%s", ch.code);
    }
  }
  if (ch.url != nullptr) {
    char norm[kUrlMax];
    if (ch.url[0] == '\0') {
      next.url[0] = '\0';
    } else if (!normalize_server_url(ch.url, norm)) {
      *err = "Sunucu adresi https:// ile baslamali ve gecerli olmali";
      return false;
    } else {
      snprintf(next.url, sizeof(next.url), "%s", norm);
    }
  }
  if (ch.clearSecret) {
    memset(next.secret, 0, sizeof(next.secret));
  } else if (ch.secret != nullptr && ch.secret[0] != '\0') {
    if (!valid_secret(ch.secret)) {
      memset(next.secret, 0, sizeof(next.secret));
      *err = "Gizli anahtar 43 karakter olmali (A-Z a-z 0-9 _ -)";
      return false;
    }
    memcpy(next.secret, ch.secret, kSecretLen);
    next.secret[kSecretLen] = '\0';
  }
  if (ch.on >= 0) next.on = (ch.on != 0);
  bool nextReq = (ch.requireStart >= 0) ? (ch.requireStart != 0) : s_requireStart;
  if (nextReq && !isActive(next)) {
    memset(next.secret, 0, sizeof(next.secret));
    *err = "Uygulamadan baslatma icin bulut acik, adres, kod ve anahtar gerekli";
    return false;
  }

  // 2) NVS'ye yaz
  Preferences p;
  if (!p.begin(kPrefsNs, false)) {
    memset(next.secret, 0, sizeof(next.secret));
    *err = "NVS acilamadi";
    return false;
  }
  bool okW = true;
  okW &= p.putBool("on", next.on) > 0;
  if (next.code[0]) okW &= p.putString("code", next.code) > 0; else rmKey(p, "code");
  if (next.url[0]) okW &= p.putString("url", next.url) > 0; else rmKey(p, "url");
  if (next.secret[0]) okW &= p.putString("secret", next.secret) > 0; else rmKey(p, "secret");
  okW &= p.putBool("reqStart", nextReq) > 0;
  bool codeChanged;
  portENTER_CRITICAL(&s_mux);
  codeChanged = strcmp(effCode(s_cfg), effCode(next)) != 0;
  portEXIT_CRITICAL(&s_mux);
  if (codeChanged) {
    // Baska istasyon: eski komut kimligi bu istasyona ait degil.
    rmKey(p, "hasLast");
    rmKey(p, "lastId");
    rmKey(p, "lastOk");
  }
  p.end();
  if (!okW) {
    memset(next.secret, 0, sizeof(next.secret));
    *err = "NVS yazilamadi";
    return false;
  }

  // 3) Uygula
  portENTER_CRITICAL(&s_mux);
  s_cfg = next;
  s_cfgGen++;
  portEXIT_CRITICAL(&s_mux);
  s_requireStart = nextReq;
  bool active = isActive(next);
  memset(next.secret, 0, sizeof(next.secret));
  Serial.printf("[CLOUD] Ayar kaydedildi: %s, kod=%s, requireCloudStart=%d%s\n",
                active ? "etkin" : "pasif", effCode(next), nextReq ? 1 : 0,
                codeChanged ? " (kod degisti; komut gecmisi sifirlandi)" : "");
  return true;
}

void cloud_status_json(char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return;
  ensureMacCode();
  portENTER_CRITICAL(&s_mux);
  bool on = s_cfg.on;
  bool secretSet = valid_secret(s_cfg.secret);
  bool active = isActive(s_cfg);
  char code[17];
  snprintf(code, sizeof(code), "%s", effCode(s_cfg));
  bool codeCustom = s_cfg.code[0] != '\0';
  char url[kUrlMax];
  snprintf(url, sizeof(url), "%s", effUrl(s_cfg));
  bool everOk = s_everOk;
  uint32_t lastOk = s_lastOkMs;
  int lastHttp = s_lastHttp;
  uint32_t failures = s_failures;
  const char* phase = s_phase;
  int64_t lastCmdId = s_lastCmdId;
  bool lastCmdOk = s_lastCmdOk;
  CmdType lastCmdType = s_lastCmdType;
  portEXIT_CRITICAL(&s_mux);
  uint32_t now = millis();
  bool online = active && everOk && (uint32_t)(now - lastOk) < kOnlineWindowMs;
  long ageSec = (active && everOk) ? (long)((now - lastOk) / 1000UL) : -1L;
  snprintf(out, outLen,
           "{\"enabled\":%s,\"on\":%s,\"secretSet\":%s,\"code\":\"%s\",\"codeDefault\":\"%s\",\"codeCustom\":%s,"
           "\"url\":\"%s\",\"ok\":%s,\"phase\":\"%s\",\"ageSec\":%ld,\"lastHttp\":%d,\"failures\":%lu,"
           "\"requireCloudStart\":%s,\"authorized\":%s,\"paused\":%s,\"lastCmdId\":%lld,\"lastCmdType\":\"%s\",\"lastCmdOk\":%s}",
           active ? "true" : "false", on ? "true" : "false", secretSet ? "true" : "false", code, s_macCode,
           codeCustom ? "true" : "false", url, online ? "true" : "false", phase, ageSec, lastHttp,
           (unsigned long)failures, (s_requireStart && active) ? "true" : "false",
           s_gate.authorized() ? "true" : "false", s_paused ? "true" : "false",
           (long long)lastCmdId, cmd_type_name(lastCmdType), lastCmdOk ? "true" : "false");
}

bool cloud_loop(const String& stableState, float ia, float ib, float ic,
                float powerW, float energyKWh, uint32_t chargeSeconds, bool schedBlocked) {
  char st = normalize_state(stableState.length() > 0 ? stableState[0] : 'F');
  uint32_t nowMs = millis();
  bool active = activeNow();
  bool require = active && s_requireStart;

  // 1) Bekleyen bulut komutu
  bool havePending = false;
  CmdType type = CmdType::None;
  portENTER_CRITICAL(&s_mux);
  if (s_cmdPending && !s_cmdDone) {
    havePending = true;
    type = s_cmdType;
  }
  portEXIT_CRITICAL(&s_mux);
  if (havePending) {
    bool ok = applyCommand(type, st, require, nowMs);
    portENTER_CRITICAL(&s_mux);
    s_cmdOk = ok;
    s_cmdDone = true;
    portEXIT_CRITICAL(&s_mux);
  }

  // 2) Bulut STOP kilidi: arac ayrilinca AUTO'ya don; baska bir yoldan mod degistiyse kilit biter.
  if (s_cloudStopLatched) {
    if (g_chargeMode != 2) {
      s_cloudStopLatched = false;
    } else if (st == 'A') {
      g_chargeMode = 0;
      s_cloudStopLatched = false;
    }
  }

  // 3) Guvenli mod kapisi (baglanti kopmasi izni KALDIRMAZ) + zamanlayici
  bool localStop = (g_chargeMode == 2);
  bool gatePermit = s_gate.update(require, st, localStop);
  bool permit = gatePermit && !schedBlocked;
  if (s_prevPermit && !permit) {
    beginSoftStop(nowMs);
    Serial.printf("[GATE] Sarj izni kalkti (durum=%c, %s)\n", st,
                  schedBlocked ? "planli sarj yasakli aralik" : "uygulamadan baslatma bekleniyor");
  }
  // Izin geri geldiyse ve STOP modu yoksa bekleyen birakma iptal (sarj kesintisiz surer).
  if (permit && !localStop) s_softStop = false;
  s_prevPermit = permit;
  serviceSoftStop(st, nowMs);

  // 4) Planli sarj nedeniyle bekleme: yetki var (bulut izni ya da AUTO), arac bagli, aralik yasakli.
  bool vehicle = (st == 'B' || st == 'C' || st == 'D');
  bool wouldCharge = require ? s_gate.authorized() : (g_chargeMode != 2);
  bool paused = schedBlocked && vehicle && wouldCharge;
  s_paused = paused;

  // 5) Telemetri (gorev bir sonraki yoklamada gonderir)
  if (active) {
    evse_sched::Eval se = sched_eval();
    portENTER_CRITICAL(&s_mux);
    s_tel.state = st;
    s_tel.ia = ia;
    s_tel.ib = ib;
    s_tel.ic = ic;
    s_tel.pW = powerW;
    s_tel.eKWh = energyKWh;
    s_tel.tSec = chargeSeconds;
    s_tel.paused = paused;
    s_tel.resumeInSec = paused ? se.nextChangeInSec : -1;
    s_tel.schedOn = se.enabled;
    s_tel.schedBlocked = schedBlocked;
    s_tel.schedTimeOk = se.timeValid;
    s_tel.schedNextChangeInSec = se.nextChangeInSec;
    s_telValid = true;
    portEXIT_CRITICAL(&s_mux);
  }
  return permit;
}
