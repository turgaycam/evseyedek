#include "auth.h"

#include <Preferences.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>
#include <string.h>

#ifndef EVSE_ADMIN_USER
#define EVSE_ADMIN_USER "admin"
#endif

// Ilk acilista (NVS'de ozet yokken) kullanilan varsayilan parola.
#ifndef EVSE_ADMIN_PASSWORD
#error "EVSE_ADMIN_PASSWORD secrets.ini icinde tanimlanmali (bkz. secrets.example.ini)"
#endif

// Parolayi unutma durumunda kurtarma: bu bayrakla derlenip USB'den yuklenen
// firmware acilista parolayi varsayilana dondurur (sonra bayraksiz tekrar yukleyin).
// -DEVSE_AUTH_RESET_PASSWORD=1

using namespace evse_auth;

namespace {

const char* kNvsNs = "evseauth";
const char* kKeySalt = "salt";
const char* kKeyHash = "hash";
const char* kKeyIter = "iter";
const char* kKeyMustChange = "mustchg";

uint8_t s_salt[kSaltLen];
uint8_t s_hash[kHashLen];
uint32_t s_iterations = kDefaultHashIterations;
bool s_mustChange = true;
bool s_ready = false;

LoginLimiter s_limiter;
SessionTable s_sessions(kSessionTtlMs);
OneShotTicket s_uploadTicket(kUploadTicketTtlMs);

void sha256Fn(const uint8_t* data, size_t len, uint8_t out[kHashLen]) {
  mbedtls_sha256_ret(data, len, out, 0);
}

void randomBytes(uint8_t* out, size_t n) {
  // Wi-Fi acikken esp_random donanim RNG'si gercek rastgeledir.
  esp_fill_random(out, n);
}

bool persist() {
  Preferences p;
  if (!p.begin(kNvsNs, false)) return false;
  bool ok = p.putBytes(kKeySalt, s_salt, kSaltLen) == kSaltLen;
  ok = ok && p.putBytes(kKeyHash, s_hash, kHashLen) == kHashLen;
  ok = ok && p.putUInt(kKeyIter, s_iterations) == sizeof(uint32_t);
  ok = ok && p.putUChar(kKeyMustChange, s_mustChange ? 1 : 0) == 1;
  p.end();
  return ok;
}

bool setPassword(const char* pw, size_t len, bool mustChange) {
  uint8_t salt[kSaltLen];
  uint8_t hash[kHashLen];
  randomBytes(salt, sizeof(salt));
  if (!derivePasswordHash(sha256Fn, salt, pw, len, kDefaultHashIterations, hash)) return false;
  memcpy(s_salt, salt, kSaltLen);
  memcpy(s_hash, hash, kHashLen);
  s_iterations = kDefaultHashIterations;
  s_mustChange = mustChange;
  memset(hash, 0, sizeof(hash));
  return persist();
}

bool parseHexToken(const String& hex, uint8_t out[kTokenLen]) {
  String h = hex;
  h.trim();
  return fromHex(h.c_str(), h.length(), out, kTokenLen);
}

bool parseBearer(const String& header, uint8_t out[kTokenLen]) {
  String h = header;
  h.trim();
  if (h.length() < 7) return false;
  if (!h.substring(0, 7).equalsIgnoreCase("Bearer ")) return false;
  return parseHexToken(h.substring(7), out);
}

void newSession(char tokenHex[kTokenHexLen + 1]) {
  uint8_t tok[kTokenLen];
  randomBytes(tok, sizeof(tok));
  s_sessions.add(tok, millis());
  toHex(tok, kTokenLen, tokenHex);
  memset(tok, 0, sizeof(tok));
}

bool passwordMatches(const char* pw) {
  if (!s_ready || !pw) return false;
  size_t len = strnlen(pw, kMaxPasswordLen + 1);
  if (len > kMaxPasswordLen) return false;
  return verifyPassword(sha256Fn, s_salt, s_hash, pw, len, s_iterations);
}

}  // namespace

void auth_init() {
  Preferences p;
  bool loaded = false;
  if (p.begin(kNvsNs, true)) {
    size_t gotSalt = p.getBytes(kKeySalt, s_salt, kSaltLen);
    size_t gotHash = p.getBytes(kKeyHash, s_hash, kHashLen);
    s_iterations = p.getUInt(kKeyIter, kDefaultHashIterations);
    s_mustChange = p.getUChar(kKeyMustChange, 1) != 0;
    p.end();
    loaded = (gotSalt == kSaltLen && gotHash == kHashLen &&
              s_iterations > 0 && s_iterations <= kMaxHashIterations);
  }
#if defined(EVSE_AUTH_RESET_PASSWORD) && EVSE_AUTH_RESET_PASSWORD
  loaded = false;
  Serial.println("[AUTH] EVSE_AUTH_RESET_PASSWORD: parola varsayilana donduruluyor");
#endif
  if (!loaded) {
    const char* def = EVSE_ADMIN_PASSWORD;
    if (!setPassword(def, strlen(def), true)) {
      Serial.println("[AUTH] UYARI: varsayilan parola NVS'ye yazilamadi (RAM'de gecerli)");
    } else {
      Serial.println("[AUTH] Varsayilan panel parolasi olusturuldu; ilk giriste degistirin");
    }
  }
  s_sessions.clear();
  s_uploadTicket.clear();
  s_ready = true;
}

AuthLoginResult auth_login(const char* user, const char* password, char tokenHex[kTokenHexLen + 1], uint32_t* info) {
  uint32_t now = millis();
  uint32_t wait = s_limiter.retryAfterSec(now);
  if (wait > 0) {
    if (info) *info = wait;
    return AuthLoginResult::Locked;
  }
  bool userOk = user && strcmp(user, EVSE_ADMIN_USER) == 0;
  bool pwOk = passwordMatches(password);  // kullanici adi yanlis olsa da ozet hesaplanir
  if (userOk && pwOk) {
    s_limiter.registerSuccess();
    newSession(tokenHex);
    return AuthLoginResult::Ok;
  }
  uint8_t left = s_limiter.registerFailure(millis());
  if (left == 0) {
    if (info) *info = s_limiter.retryAfterSec(millis());
    Serial.println("[AUTH] Cok fazla hatali giris; panel girisi gecici olarak kilitlendi");
    return AuthLoginResult::Locked;
  }
  if (info) *info = left;
  return AuthLoginResult::BadPassword;
}

bool auth_check_bearer(const String& authorizationHeader) {
  uint8_t tok[kTokenLen];
  if (!parseBearer(authorizationHeader, tok)) return false;
  return s_sessions.validate(tok, millis(), true);
}

void auth_logout(const String& authorizationHeader) {
  uint8_t tok[kTokenLen];
  if (parseBearer(authorizationHeader, tok)) s_sessions.remove(tok);
}

AuthPwResult auth_change_password(const char* current, const char* next, char tokenHex[kTokenHexLen + 1], uint32_t* info) {
  uint32_t now = millis();
  uint32_t wait = s_limiter.retryAfterSec(now);
  if (wait > 0) {
    if (info) *info = wait;
    return AuthPwResult::Locked;
  }
  if (!passwordMatches(current)) {
    // Calinmis bir oturumla parola tahmini de ayni kilit sayacina tabidir.
    if (s_limiter.registerFailure(millis()) == 0) {
      if (info) *info = s_limiter.retryAfterSec(millis());
      return AuthPwResult::Locked;
    }
    return AuthPwResult::WrongCurrent;
  }
  size_t len = next ? strnlen(next, kMaxPasswordLen + 1) : 0;
  if (len < kMinPasswordLen) return AuthPwResult::TooShort;
  if (len > kMaxPasswordLen) return AuthPwResult::TooLong;
  if (strcmp(current, next) == 0) return AuthPwResult::SameAsCurrent;
  if (!setPassword(next, len, false)) return AuthPwResult::StoreFailed;
  s_limiter.registerSuccess();
  s_sessions.clear();
  s_uploadTicket.clear();
  newSession(tokenHex);
  Serial.println("[AUTH] Panel parolasi degistirildi; diger oturumlar kapatildi");
  return AuthPwResult::Ok;
}

void auth_issue_upload_ticket(char tokenHex[kTokenHexLen + 1]) {
  uint8_t tok[kTokenLen];
  randomBytes(tok, sizeof(tok));
  s_uploadTicket.issue(tok, millis());
  toHex(tok, kTokenLen, tokenHex);
  memset(tok, 0, sizeof(tok));
}

bool auth_check_upload_ticket(const String& ticketHex, bool consume) {
  uint8_t tok[kTokenLen];
  if (!parseHexToken(ticketHex, tok)) return false;
  return s_uploadTicket.check(tok, millis(), consume);
}

bool auth_must_change_password() { return s_mustChange; }

uint32_t auth_session_ttl_sec() { return kSessionTtlMs / 1000UL; }
