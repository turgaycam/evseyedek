// Host (native) birim testleri: src/net/auth_core.cpp
// Calistirma: pio test -e native
// Kapsam: SHA-256 dogrulugu, tuzlu ozet, sabit-zamanli karsilastirma, hex,
// giris kilidi (5 hata -> 60 sn, artan), oturum suresi (kayan 15 dk), 4 oturum siniri,
// tek kullanimlik /update bileti, millis() tasmasi.

#include <unity.h>
#include <stdint.h>
#include <string.h>

#include "net/auth_core.h"

using namespace evse_auth;

// ---- Testler icin kucuk SHA-256 (FIPS 180-4). Cihazda mbedtls kullanilir. ----
namespace {
const uint32_t K[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline uint32_t ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

void block(uint32_t h[8], const uint8_t* p) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i)
    w[i] = (uint32_t)p[i * 4] << 24 | (uint32_t)p[i * 4 + 1] << 16 | (uint32_t)p[i * 4 + 2] << 8 | p[i * 4 + 3];
  for (int i = 16; i < 64; ++i) {
    uint32_t s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3);
    uint32_t s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
  for (int i = 0; i < 64; ++i) {
    uint32_t t1 = hh + (ror(e, 6) ^ ror(e, 11) ^ ror(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
    uint32_t t2 = (ror(a, 2) ^ ror(a, 13) ^ ror(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
    hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
  }
  h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

void sha256(const uint8_t* data, size_t len, uint8_t out[32]) {
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  size_t i = 0;
  for (; i + 64 <= len; i += 64) block(h, data + i);
  uint8_t tail[128] = {0};
  size_t rem = len - i;
  memcpy(tail, data + i, rem);
  tail[rem] = 0x80;
  size_t tl = (rem + 9 <= 64) ? 64 : 128;
  uint64_t bits = (uint64_t)len * 8;
  for (int k = 0; k < 8; ++k) tail[tl - 1 - k] = (uint8_t)(bits >> (8 * k));
  block(h, tail);
  if (tl == 128) block(h, tail + 64);
  for (int k = 0; k < 8; ++k) {
    out[k * 4] = h[k] >> 24; out[k * 4 + 1] = h[k] >> 16; out[k * 4 + 2] = h[k] >> 8; out[k * 4 + 3] = h[k];
  }
}

void tok(uint8_t t[kTokenLen], uint8_t seed) {
  for (size_t i = 0; i < kTokenLen; ++i) t[i] = (uint8_t)(seed * 31 + i);
}
}  // namespace

void setUp() {}
void tearDown() {}

void test_sha256_vectors() {
  uint8_t out[32];
  char hex[65];
  sha256((const uint8_t*)"abc", 3, out);
  toHex(out, 32, hex);
  TEST_ASSERT_EQUAL_STRING("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", hex);
  const char* m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  sha256((const uint8_t*)m, strlen(m), out);
  toHex(out, 32, hex);
  TEST_ASSERT_EQUAL_STRING("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1", hex);
}

void test_hash_single_iteration_is_sha_of_salt_pw() {
  uint8_t salt[kSaltLen];
  memset(salt, 0xAB, sizeof(salt));
  uint8_t a[32], b[32], buf[kSaltLen + 10];
  TEST_ASSERT_TRUE(derivePasswordHash(sha256, salt, "rotosis123", 10, 1, a));
  memcpy(buf, salt, kSaltLen);
  memcpy(buf + kSaltLen, "rotosis123", 10);
  sha256(buf, sizeof(buf), b);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(b, a, 32);
}

void test_hash_verify_and_salt() {
  uint8_t s1[kSaltLen], s2[kSaltLen];
  memset(s1, 1, sizeof(s1));
  memset(s2, 2, sizeof(s2));
  uint8_t h1[32], h2[32];
  TEST_ASSERT_TRUE(derivePasswordHash(sha256, s1, "rotosis123", 10, kDefaultHashIterations, h1));
  TEST_ASSERT_TRUE(derivePasswordHash(sha256, s2, "rotosis123", 10, kDefaultHashIterations, h2));
  TEST_ASSERT_FALSE(memcmp(h1, h2, 32) == 0);  // farkli tuz -> farkli ozet
  TEST_ASSERT_TRUE(verifyPassword(sha256, s1, h1, "rotosis123", 10, kDefaultHashIterations));
  TEST_ASSERT_FALSE(verifyPassword(sha256, s1, h1, "rotosis124", 10, kDefaultHashIterations));
  TEST_ASSERT_FALSE(verifyPassword(sha256, s1, h1, "rotosis12", 9, kDefaultHashIterations));
  TEST_ASSERT_FALSE(verifyPassword(sha256, s1, h1, "rotosis123", 10, kDefaultHashIterations + 1));
  char longPw[kMaxPasswordLen + 2];
  memset(longPw, 'x', sizeof(longPw));
  TEST_ASSERT_FALSE(derivePasswordHash(sha256, s1, longPw, kMaxPasswordLen + 1, 1, h2));
  TEST_ASSERT_TRUE(derivePasswordHash(sha256, s1, longPw, kMaxPasswordLen, 1, h2));
  TEST_ASSERT_FALSE(derivePasswordHash(sha256, s1, "a", 1, 0, h2));
}

void test_const_time_eq_and_hex() {
  uint8_t a[4] = {1, 2, 3, 4}, b[4] = {1, 2, 3, 4}, c[4] = {1, 2, 3, 5};
  TEST_ASSERT_TRUE(constTimeEq(a, b, 4));
  TEST_ASSERT_FALSE(constTimeEq(a, c, 4));
  char hex[9];
  toHex(c, 4, hex);
  TEST_ASSERT_EQUAL_STRING("01020305", hex);
  uint8_t back[4];
  TEST_ASSERT_TRUE(fromHex("01020305", 8, back, 4));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(c, back, 4);
  TEST_ASSERT_TRUE(fromHex("0A0b0C0d", 8, back, 4));
  TEST_ASSERT_FALSE(fromHex("0102030", 7, back, 4));
  TEST_ASSERT_FALSE(fromHex("010203zz", 8, back, 4));
}

void test_limiter_locks_after_five() {
  LoginLimiter l;
  uint32_t t = 1000;
  TEST_ASSERT_EQUAL_UINT32(0, l.retryAfterSec(t));
  TEST_ASSERT_EQUAL_UINT8(4, l.registerFailure(t));
  TEST_ASSERT_EQUAL_UINT8(3, l.registerFailure(t));
  TEST_ASSERT_EQUAL_UINT8(2, l.registerFailure(t));
  TEST_ASSERT_EQUAL_UINT8(1, l.registerFailure(t));
  TEST_ASSERT_EQUAL_UINT8(0, l.registerFailure(t));  // 5. hata -> kilit
  TEST_ASSERT_EQUAL_UINT32(60, l.retryAfterSec(t));
  TEST_ASSERT_EQUAL_UINT32(30, l.retryAfterSec(t + 30000));
  TEST_ASSERT_EQUAL_UINT8(0, l.registerFailure(t + 30000));  // kilitliyken deneme sayilmaz
  TEST_ASSERT_EQUAL_UINT32(30, l.retryAfterSec(t + 30000));
  TEST_ASSERT_EQUAL_UINT32(0, l.retryAfterSec(t + 60000));   // kilit bitti
  TEST_ASSERT_EQUAL_UINT8(4, l.registerFailure(t + 60001));  // yeni pencere
}

void test_limiter_escalates_and_resets() {
  LoginLimiter l;
  uint32_t t = 0;
  uint32_t expect[] = {60, 120, 240, 480, 900, 900};
  for (int round = 0; round < 6; ++round) {
    for (int i = 0; i < 5; ++i) l.registerFailure(t);
    TEST_ASSERT_EQUAL_UINT32(expect[round], l.retryAfterSec(t));
    t += expect[round] * 1000UL;
    TEST_ASSERT_EQUAL_UINT32(0, l.retryAfterSec(t));
  }
  l.registerSuccess();
  for (int i = 0; i < 5; ++i) l.registerFailure(t);
  TEST_ASSERT_EQUAL_UINT32(60, l.retryAfterSec(t));  // basari seviyeyi sifirlar
}

void test_limiter_millis_wrap() {
  LoginLimiter l;
  uint32_t t = 0xFFFFFFFFUL - 10000UL;  // 10 sn sonra millis() tasar
  for (int i = 0; i < 5; ++i) l.registerFailure(t);
  TEST_ASSERT_EQUAL_UINT32(60, l.retryAfterSec(t));
  TEST_ASSERT_EQUAL_UINT32(40, l.retryAfterSec(t + 20000UL));  // tasma sonrasi
  TEST_ASSERT_EQUAL_UINT32(0, l.retryAfterSec(t + 60000UL));
}

void test_session_sliding_ttl() {
  SessionTable s(kSessionTtlMs);
  uint8_t a[kTokenLen], x[kTokenLen];
  tok(a, 1);
  tok(x, 9);
  s.add(a, 1000);
  TEST_ASSERT_TRUE(s.validate(a, 1000 + kSessionTtlMs - 1));  // kayar
  TEST_ASSERT_TRUE(s.validate(a, 1000 + 2 * kSessionTtlMs - 2));
  TEST_ASSERT_FALSE(s.validate(x, 1000));
  TEST_ASSERT_FALSE(s.validate(a, 1000 + 3 * kSessionTtlMs));  // 15 dk islemsiz -> dustu
  TEST_ASSERT_EQUAL_INT(0, s.activeCount(1000 + 3 * kSessionTtlMs));
}

void test_session_no_touch() {
  SessionTable s(1000);
  uint8_t a[kTokenLen];
  tok(a, 1);
  s.add(a, 0);
  TEST_ASSERT_TRUE(s.validate(a, 900, false));
  TEST_ASSERT_FALSE(s.validate(a, 1000, false));
}

void test_session_limit_four_evicts_oldest() {
  SessionTable s;
  uint8_t t[6][kTokenLen];
  for (int i = 0; i < 6; ++i) tok(t[i], (uint8_t)(i + 1));
  s.add(t[0], 100);
  s.add(t[1], 200);
  s.add(t[2], 300);
  s.add(t[3], 400);
  TEST_ASSERT_EQUAL_INT(4, s.activeCount(500));
  TEST_ASSERT_TRUE(s.validate(t[0], 500));  // t0 tazelendi, en eski artik t1
  s.add(t[4], 600);
  TEST_ASSERT_EQUAL_INT(4, s.activeCount(600));
  TEST_ASSERT_FALSE(s.validate(t[1], 600));
  TEST_ASSERT_TRUE(s.validate(t[0], 600));
  TEST_ASSERT_TRUE(s.validate(t[4], 600));
  // Yuzlerce giris tabloyu buyutmez.
  for (int i = 0; i < 300; ++i) {
    uint8_t z[kTokenLen];
    tok(z, (uint8_t)i);
    s.add(z, 700 + i);
  }
  TEST_ASSERT_EQUAL_INT(kMaxSessions, s.activeCount(1000));
}

void test_session_remove_and_clear() {
  SessionTable s;
  uint8_t a[kTokenLen], b[kTokenLen];
  tok(a, 1);
  tok(b, 2);
  s.add(a, 0);
  s.add(b, 0);
  TEST_ASSERT_TRUE(s.remove(a));
  TEST_ASSERT_FALSE(s.validate(a, 1));
  TEST_ASSERT_TRUE(s.validate(b, 1));
  s.clear();
  TEST_ASSERT_FALSE(s.validate(b, 2));
  uint8_t zero[kTokenLen] = {0};
  TEST_ASSERT_FALSE(s.validate(zero, 2));  // bos yuva sifir jetonla eslesmez
}

void test_upload_ticket_one_shot_and_expiry() {
  OneShotTicket t(kUploadTicketTtlMs);
  uint8_t a[kTokenLen], b[kTokenLen];
  tok(a, 3);
  tok(b, 4);
  uint8_t zero[kTokenLen] = {0};
  TEST_ASSERT_FALSE(t.check(zero, 0, false));
  t.issue(a, 1000);
  TEST_ASSERT_FALSE(t.check(b, 1500, true));
  TEST_ASSERT_TRUE(t.check(a, 1500, false));  // sayfa acilisi: tuketmez
  TEST_ASSERT_TRUE(t.check(a, 2000, true));   // yukleme: tuketir
  TEST_ASSERT_FALSE(t.check(a, 2001, true));  // ikinci kez gecmez
  t.issue(a, 5000);
  TEST_ASSERT_FALSE(t.check(a, 5000 + kUploadTicketTtlMs, true));  // 60 sn doldu
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_sha256_vectors);
  RUN_TEST(test_hash_single_iteration_is_sha_of_salt_pw);
  RUN_TEST(test_hash_verify_and_salt);
  RUN_TEST(test_const_time_eq_and_hex);
  RUN_TEST(test_limiter_locks_after_five);
  RUN_TEST(test_limiter_escalates_and_resets);
  RUN_TEST(test_limiter_millis_wrap);
  RUN_TEST(test_session_sliding_ttl);
  RUN_TEST(test_session_no_touch);
  RUN_TEST(test_session_limit_four_evicts_oldest);
  RUN_TEST(test_session_remove_and_clear);
  RUN_TEST(test_upload_ticket_one_shot_and_expiry);
  return UNITY_END();
}
