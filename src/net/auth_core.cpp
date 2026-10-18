#include "auth_core.h"

#include <string.h>

namespace evse_auth {

bool constTimeEq(const uint8_t* a, const uint8_t* b, size_t n) {
  volatile uint8_t diff = 0;
  for (size_t i = 0; i < n; ++i) diff |= (uint8_t)(a[i] ^ b[i]);
  return diff == 0;
}

static void wipe(void* p, size_t n) {
  volatile uint8_t* v = (volatile uint8_t*)p;
  while (n--) *v++ = 0;
}

bool derivePasswordHash(Sha256Fn sha,
                        const uint8_t* salt,
                        const char* pw,
                        size_t pwLen,
                        uint32_t iterations,
                        uint8_t out[kHashLen]) {
  if (!sha || !salt || !out || (!pw && pwLen) || pwLen > kMaxPasswordLen) return false;
  if (iterations == 0 || iterations > kMaxHashIterations) return false;

  uint8_t buf[kHashLen + kSaltLen + kMaxPasswordLen];
  // h0 = H(salt | pw)
  memcpy(buf, salt, kSaltLen);
  if (pwLen) memcpy(buf + kSaltLen, pw, pwLen);
  sha(buf, kSaltLen + pwLen, out);

  // hi = H(h(i-1) | salt | pw)
  memcpy(buf + kHashLen, salt, kSaltLen);
  if (pwLen) memcpy(buf + kHashLen + kSaltLen, pw, pwLen);
  for (uint32_t i = 1; i < iterations; ++i) {
    memcpy(buf, out, kHashLen);
    sha(buf, kHashLen + kSaltLen + pwLen, out);
  }
  wipe(buf, sizeof(buf));
  return true;
}

bool verifyPassword(Sha256Fn sha,
                    const uint8_t* salt,
                    const uint8_t* storedHash,
                    const char* pw,
                    size_t pwLen,
                    uint32_t iterations) {
  if (!storedHash) return false;
  uint8_t calc[kHashLen];
  bool ok = derivePasswordHash(sha, salt, pw, pwLen, iterations, calc);
  bool eq = constTimeEq(calc, storedHash, kHashLen);
  wipe(calc, sizeof(calc));
  return ok && eq;
}

void toHex(const uint8_t* in, size_t n, char* out) {
  static const char kHex[] = "0123456789abcdef";
  for (size_t i = 0; i < n; ++i) {
    out[i * 2] = kHex[in[i] >> 4];
    out[i * 2 + 1] = kHex[in[i] & 0x0F];
  }
  out[n * 2] = '\0';
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool fromHex(const char* in, size_t inLen, uint8_t* out, size_t n) {
  if (!in || !out || inLen != n * 2) return false;
  for (size_t i = 0; i < n; ++i) {
    int hi = hexVal(in[i * 2]);
    int lo = hexVal(in[i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

// ---- LoginLimiter ----

uint32_t LoginLimiter::retryAfterSec(uint32_t now) {
  if (!locked_) return 0;
  if (timeReached(now, lockUntil_)) {
    // Kilit bitti: yeni bir 5 denemelik pencere acilir, seviye korunur.
    locked_ = false;
    fails_ = 0;
    return 0;
  }
  uint32_t leftMs = lockUntil_ - now;
  return (leftMs + 999UL) / 1000UL;
}

uint8_t LoginLimiter::registerFailure(uint32_t now) {
  if (retryAfterSec(now) > 0) return 0;
  if (fails_ < kMaxFails) fails_++;
  if (fails_ >= kMaxFails) {
    uint32_t dur = kBaseLockMs;
    for (uint8_t i = 0; i < level_ && dur < kMaxLockMs; ++i) dur *= 2;
    if (dur > kMaxLockMs) dur = kMaxLockMs;
    if (level_ < 8) level_++;
    locked_ = true;
    lockUntil_ = now + dur;
    return 0;
  }
  return (uint8_t)(kMaxFails - fails_);
}

void LoginLimiter::registerSuccess() {
  fails_ = 0;
  level_ = 0;
  locked_ = false;
  lockUntil_ = 0;
}

// ---- SessionTable ----

SessionTable::SessionTable(uint32_t ttlMs) : ttl_(ttlMs) { clear(); }

bool SessionTable::expired(const TokenSlot& s, uint32_t now) const {
  return (uint32_t)(now - s.stampMs) >= ttl_;
}

void SessionTable::add(const uint8_t tok[kTokenLen], uint32_t now) {
  int target = -1;
  // 1) bos ya da suresi dolmus yuva
  for (int i = 0; i < kMaxSessions; ++i) {
    if (!slots_[i].used || expired(slots_[i], now)) {
      target = i;
      break;
    }
  }
  // 2) yoksa en uzun suredir kullanilmayan
  if (target < 0) {
    uint32_t oldestAge = 0;
    target = 0;
    for (int i = 0; i < kMaxSessions; ++i) {
      uint32_t age = now - slots_[i].stampMs;
      if (age >= oldestAge) {
        oldestAge = age;
        target = i;
      }
    }
  }
  memcpy(slots_[target].tok, tok, kTokenLen);
  slots_[target].stampMs = now;
  slots_[target].used = true;
}

bool SessionTable::validate(const uint8_t tok[kTokenLen], uint32_t now, bool touch) {
  int hit = -1;
  // Tum yuvalar her seferinde karsilastirilir (zamanlama esitligi).
  for (int i = 0; i < kMaxSessions; ++i) {
    bool eq = constTimeEq(slots_[i].tok, tok, kTokenLen);
    if (slots_[i].used && expired(slots_[i], now)) {
      memset(slots_[i].tok, 0, kTokenLen);
      slots_[i].used = false;
      continue;
    }
    if (slots_[i].used && eq && hit < 0) hit = i;
  }
  if (hit < 0) return false;
  if (touch) slots_[hit].stampMs = now;
  return true;
}

bool SessionTable::remove(const uint8_t tok[kTokenLen]) {
  bool removed = false;
  for (int i = 0; i < kMaxSessions; ++i) {
    if (slots_[i].used && constTimeEq(slots_[i].tok, tok, kTokenLen)) {
      memset(slots_[i].tok, 0, kTokenLen);
      slots_[i].used = false;
      removed = true;
    }
  }
  return removed;
}

void SessionTable::clear() {
  for (int i = 0; i < kMaxSessions; ++i) {
    memset(slots_[i].tok, 0, kTokenLen);
    slots_[i].stampMs = 0;
    slots_[i].used = false;
  }
}

int SessionTable::activeCount(uint32_t now) {
  int n = 0;
  for (int i = 0; i < kMaxSessions; ++i) {
    if (slots_[i].used && !expired(slots_[i], now)) n++;
  }
  return n;
}

// ---- OneShotTicket ----

OneShotTicket::OneShotTicket(uint32_t ttlMs) : ttl_(ttlMs) { clear(); }

void OneShotTicket::issue(const uint8_t tok[kTokenLen], uint32_t now) {
  memcpy(slot_.tok, tok, kTokenLen);
  slot_.stampMs = now;
  slot_.used = true;
}

bool OneShotTicket::check(const uint8_t tok[kTokenLen], uint32_t now, bool consume) {
  bool eq = constTimeEq(slot_.tok, tok, kTokenLen);
  if (!slot_.used) return false;
  if ((uint32_t)(now - slot_.stampMs) >= ttl_) {
    clear();
    return false;
  }
  if (!eq) return false;
  if (consume) clear();
  return true;
}

void OneShotTicket::clear() {
  memset(slot_.tok, 0, kTokenLen);
  slot_.stampMs = 0;
  slot_.used = false;
}

}  // namespace evse_auth
