#pragma once

// Sarj sunucusu (sarj.rotosis.com) bulut istemcisinin donanimdan bagimsiz cekirdegi.
// Arduino/ESP32 bagimliligi yoktur; host'ta (pio test -e native) birim testiyle dogrulanir.
// Cihaz tarafindaki gorev, HTTPS ve role/PWM baglantisi cloud_client.cpp icindedir.
//
// Kapsam:
// - Yoklama araligi ve hata durumunda ustel geri cekilme
// - /api/device/poll istek JSON'unu kurma, yanit JSON'unu ayristirma
// - Komut tekrar korumasi (ayni id ikinci kez uygulanmaz, yalnizca yeniden onaylanir)
// - Guvenli mod kapisi (requireCloudStart): sarj yalnizca BASLARKEN bulut "start" ister;
//   basladiktan sonra baglanti kopsa da sarj kesilmez.

#include <stddef.h>
#include <stdint.h>

namespace evse_cloud {

constexpr uint32_t kPollIntervalMs = 2500;     // normal yoklama araligi
constexpr uint32_t kBackoffMaxMs = 30000;      // geri cekilme ust siniri
constexpr uint32_t kSoftStopMaxWaitMs = 3000;  // PWM kapandiktan sonra role icin en fazla bekleme

// Ardisik hata sayisina gore bir sonraki yoklamaya kadar beklenecek sure.
// 0 -> 2500 ms; n -> 2500 * 2^n, en fazla 30000 ms.
uint32_t backoff_delay_ms(uint32_t consecutiveFailures);

// Kod / gizli anahtar yalnizca [A-Za-z0-9_-] ve 1..64 karakter olabilir (JSON kacisi gerekmez).
bool is_safe_token(const char* s);

// Panelden girilen bulut ayarlarinin dogrulamasi (NVS'ye yazilmadan once).
// Istasyon kodu: 3-16 karakter [A-Z0-9_-] (kucuk harf kabul edilmez; cagiran buyutur).
bool valid_station_code(const char* s);
// Gizli anahtar: tam 43 karakter base64url [A-Za-z0-9_-].
constexpr size_t kSecretLen = 43;
bool valid_secret(const char* s);
// Sunucu adresi: yalnizca https://, host [A-Za-z0-9.-], istege bagli :port ve /yol; en cok 95 karakter.
// Sondaki '/' atilir. Gecerliyse out'a normal bicimi yazar.
constexpr size_t kUrlMax = 96;
bool normalize_server_url(const char* in, char out[kUrlMax]);

// IEC 61851 durum harfi; A-F disi her sey 'F' (EVSE hatasi) olarak raporlanir.
char normalize_state(char c);

struct Telemetry {
  char state = 'A';
  float ia = 0.0f;
  float ib = 0.0f;
  float ic = 0.0f;
  float pW = 0.0f;
  float eKWh = 0.0f;
  uint32_t tSec = 0;
  // Planli sarj (zamanlayici) bilgisi
  bool paused = false;          // sarj yetkisi var ama zamanlayici nedeniyle bekliyor
  int32_t resumeInSec = -1;     // paused iken yeniden baslamaya kalan sn (-1: bilinmiyor)
  bool schedOn = false;
  bool schedBlocked = false;
  bool schedTimeOk = false;
  int32_t schedNextChangeInSec = -1;
};

struct Ack {
  bool has = false;
  int64_t id = 0;
  bool ok = false;
};

// Istek govdesini kurar. Donus: yazilan uzunluk; tampon yetmezse ya da kod/anahtar
// gecersizse -1. Gizli anahtar yalnizca bu tampona yazilir (asla gunluge degil).
int build_poll_json(char* out, size_t outLen, const char* code, const char* secret,
                    const Telemetry& t, const char* fw, const Ack& ack);

enum class CmdType : uint8_t { None = 0, Start, Stop, SetLimit, Unknown };

struct PollResponse {
  bool ok = false;
  int64_t t = 0;        // sunucu saati (unix sn)
  bool hasCmd = false;
  int64_t cmdId = 0;
  CmdType type = CmdType::None;
};

// Yanit JSON'unu ayristirir. Bicim bozuksa false.
bool parse_poll_response(const char* body, size_t len, PollResponse& out);

const char* cmd_type_name(CmdType t);

// Komut tekrar korumasi ve onay (ack) takibi.
// Sunucu onaylanmayan komutu yeniden gonderir; ayni id yalnizca bir kez uygulanir.
class CommandTracker {
 public:
  enum class Decision : uint8_t {
    Apply,     // yeni komut: ana donguye uygulat
    ReAck,     // daha once uygulandi: yalnizca onceki sonucu yeniden onayla
    InFlight,  // ayni komut su an uygulaniyor: bekle
    Ignore     // baska bir komut uygulanirken gelen yeni komut: sonra yeniden gelir
  };

  // NVS'den geri yukleme (yeniden baslatma sonrasi ayni id tekrar uygulanmasin).
  void restore(bool hasLast, int64_t lastId, bool lastOk);

  Decision on_command(int64_t id);
  // Ana dongu komutu uyguladi; sonuc bir sonraki yoklamada onaylanir.
  void on_applied(int64_t id, bool ok);
  // Ack bekleyen var mi?
  Ack pending_ack() const { return ack_; }
  // Ack iceren yoklama basariyla teslim edildi.
  void on_ack_delivered(int64_t id);

  bool has_last() const { return hasLast_; }
  int64_t last_id() const { return lastId_; }
  bool last_ok() const { return lastOk_; }
  bool in_flight() const { return inFlight_; }

 private:
  bool hasLast_ = false;
  int64_t lastId_ = 0;
  bool lastOk_ = false;
  bool inFlight_ = false;
  int64_t inFlightId_ = 0;
  Ack ack_;
};

// Guvenli mod kapisi. requireCloudStart kapaliyken her zaman izin verir (mevcut davranis).
// Aciksa: sarjin BASLAMASI icin bulut "start" komutu gerekir. Izni yalnizca bulut "stop",
// arac ayrilmasi (A) ve yerel panelden durdurma (STOP modu) kaldirir. Bulut/Wi-Fi baglantisinin
// kopmasi izni KALDIRMAZ (sarj kesilmez). Zamanlayici izni etkilemez, yalnizca bekletir.
// Izin RAM'dedir; yeniden baslatmada kalkar.
class SafeGate {
 public:
  // "start" komutu. Donus = ackOk. Arac B/C/D degilse reddedilir.
  bool on_start(char state, bool requireCloudStart);
  // "stop" komutu. Her zaman basarili.
  bool on_stop();
  // Her ana dongu turunda cagrilir; sarja izin var mi? localStop: panelden STOP modu aktif.
  bool update(bool requireCloudStart, char state, bool localStop);
  bool authorized() const { return authorized_; }

 private:
  bool authorized_ = false;
};

}  // namespace evse_cloud
