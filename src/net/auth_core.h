#pragma once

// Yonetim paneli kimlik dogrulamasinin donanimdan bagimsiz cekirdegi.
// Arduino / ESP-IDF basligi icermez; boylece host (native) testlerinde de derlenir.
// Cihaza ozel kisimlar (NVS, mbedtls, esp_random, HTTP) auth.cpp icindedir.

#include <stddef.h>
#include <stdint.h>

namespace evse_auth {

constexpr size_t kTokenLen = 16;            // 128 bit oturum jetonu
constexpr size_t kTokenHexLen = kTokenLen * 2;
constexpr size_t kSaltLen = 16;
constexpr size_t kHashLen = 32;             // SHA-256
constexpr size_t kMinPasswordLen = 10;
constexpr size_t kMaxPasswordLen = 64;
constexpr int kMaxSessions = 4;             // RAM'deki en fazla oturum
constexpr uint32_t kSessionTtlMs = 15UL * 60UL * 1000UL;   // kayan sure
constexpr uint32_t kUploadTicketTtlMs = 60UL * 1000UL;     // /update tek kullanimlik jeton
constexpr uint8_t kMaxFails = 5;            // kilitten once hatali deneme hakki
constexpr uint32_t kBaseLockMs = 60UL * 1000UL;            // ilk kilit 60 sn
constexpr uint32_t kMaxLockMs = 15UL * 60UL * 1000UL;      // kilit en fazla 15 dk
constexpr uint32_t kDefaultHashIterations = 1000;
constexpr uint32_t kMaxHashIterations = 100000;

typedef void (*Sha256Fn)(const uint8_t* data, size_t len, uint8_t out[kHashLen]);

// millis() tasmasina dayanikli "deadline gecti mi" kontrolu.
inline bool timeReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

// Uzunluk sabitken icerikten bagimsiz surede karsilastirir.
bool constTimeEq(const uint8_t* a, const uint8_t* b, size_t n);

// Tuzlu, yinelemeli SHA-256: h0 = H(salt|pw), hi = H(h(i-1)|salt|pw).
// pwLen > kMaxPasswordLen veya iterations 0 ise false.
bool derivePasswordHash(Sha256Fn sha,
                        const uint8_t* salt,
                        const char* pw,
                        size_t pwLen,
                        uint32_t iterations,
                        uint8_t out[kHashLen]);

bool verifyPassword(Sha256Fn sha,
                    const uint8_t* salt,
                    const uint8_t* storedHash,
                    const char* pw,
                    size_t pwLen,
                    uint32_t iterations);

// out en az 2n+1 bayt olmali (sonuna '\0' yazilir), kucuk harf hex.
void toHex(const uint8_t* in, size_t n, char* out);
// inLen tam olarak 2n olmali; gecersiz karakterde false.
bool fromHex(const char* in, size_t inLen, uint8_t* out, size_t n);

// Global (cihaz geneli) hatali giris sayaci. Sabit boyutludur; saldiri altinda bellek buyumez.
// 5 hatada kilit: 60 sn, sonraki her kilitte iki katina cikar (en fazla 15 dk).
// Basarili giriste sayac ve kilit seviyesi sifirlanir.
class LoginLimiter {
 public:
  // 0: giris serbest; >0: kalan kilit suresi (sn, yukari yuvarlanir).
  uint32_t retryAfterSec(uint32_t now);
  // Hatali denemeyi kaydeder. Donus: kalan hak (>0) ya da 0 = simdi kilitlendi.
  uint8_t registerFailure(uint32_t now);
  void registerSuccess();
  uint8_t level() const { return level_; }
  uint8_t fails() const { return fails_; }

 private:
  uint8_t fails_ = 0;
  uint8_t level_ = 0;
  bool locked_ = false;
  uint32_t lockUntil_ = 0;
};

struct TokenSlot {
  uint8_t tok[kTokenLen];
  uint32_t stampMs;
  bool used;
};

// Sabit 4 yuvali oturum tablosu. Doluysa en uzun suredir kullanilmayan atilir.
class SessionTable {
 public:
  explicit SessionTable(uint32_t ttlMs = kSessionTtlMs);
  void add(const uint8_t tok[kTokenLen], uint32_t now);
  // Gecerliyse true ve (touch ise) son kullanim zamanini yeniler (kayan sure).
  bool validate(const uint8_t tok[kTokenLen], uint32_t now, bool touch = true);
  bool remove(const uint8_t tok[kTokenLen]);
  void clear();
  int activeCount(uint32_t now);

 private:
  bool expired(const TokenSlot& s, uint32_t now) const;
  TokenSlot slots_[kMaxSessions];
  uint32_t ttl_;
};

// /update icin kisa omurlu, tek kullanimlik bilet.
class OneShotTicket {
 public:
  explicit OneShotTicket(uint32_t ttlMs = kUploadTicketTtlMs);
  void issue(const uint8_t tok[kTokenLen], uint32_t now);
  // consume true ise basarili kontrolden sonra bilet iptal edilir.
  bool check(const uint8_t tok[kTokenLen], uint32_t now, bool consume);
  void clear();

 private:
  TokenSlot slot_;
  uint32_t ttl_;
};

}  // namespace evse_auth
