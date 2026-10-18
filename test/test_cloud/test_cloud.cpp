// Host (native) birim testleri: src/net/cloud_core.cpp ve src/net/schedule_core.cpp
// Calistirma: pio test -e native
// Kapsam: geri cekilme, ayar dogrulama, yoklama JSON kurma/ayristirma, komut tekrar korumasi,
// guvenli mod kapisi (baslarken bulut onayi; baglanti kopmasi sarji kesmez),
// planli sarj zamanlayicisi (aralik hesabi, gece yarisi asimi, gun secimi, saat gecersiz).

#include <unity.h>
#include <ArduinoJson.h>
#include <stdint.h>
#include <string.h>

#include "net/cloud_core.h"
#include "net/schedule_core.h"

using namespace evse_cloud;

void setUp() {}
void tearDown() {}

static const char* kTestSecret = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJ0123-_Z";  // 43 krk, sahte

// ---------------- Bulut ----------------

static void test_backoff() {
  TEST_ASSERT_EQUAL_UINT32(2500, backoff_delay_ms(0));
  TEST_ASSERT_EQUAL_UINT32(5000, backoff_delay_ms(1));
  TEST_ASSERT_EQUAL_UINT32(10000, backoff_delay_ms(2));
  TEST_ASSERT_EQUAL_UINT32(20000, backoff_delay_ms(3));
  TEST_ASSERT_EQUAL_UINT32(30000, backoff_delay_ms(4));
  TEST_ASSERT_EQUAL_UINT32(30000, backoff_delay_ms(50));
  TEST_ASSERT_EQUAL_UINT32(30000, backoff_delay_ms(0xFFFFFFFFu));
}

static void test_config_validation() {
  TEST_ASSERT_EQUAL_size_t(43, strlen(kTestSecret));
  TEST_ASSERT_TRUE(valid_secret(kTestSecret));
  TEST_ASSERT_FALSE(valid_secret("kisa"));
  TEST_ASSERT_FALSE(valid_secret("abcdefghijklmnopqrstuvwxyzABCDEFGHIJ0123-_Z9"));   // 44
  TEST_ASSERT_FALSE(valid_secret("abcdefghijklmnopqrstuvwxyzABCDEFGHIJ0123+/="));    // base64 (url degil)
  TEST_ASSERT_FALSE(valid_secret(nullptr));

  TEST_ASSERT_TRUE(valid_station_code("4571A4"));
  TEST_ASSERT_TRUE(valid_station_code("IST_01-A"));
  TEST_ASSERT_FALSE(valid_station_code("AB"));
  TEST_ASSERT_FALSE(valid_station_code("ABCDEFGHIJKLMNOPQ"));  // 17
  TEST_ASSERT_FALSE(valid_station_code("ist01"));              // kucuk harf
  TEST_ASSERT_FALSE(valid_station_code("IST 01"));
  TEST_ASSERT_FALSE(valid_station_code("IST\"01"));

  char out[kUrlMax];
  TEST_ASSERT_TRUE(normalize_server_url("https://sarj.rotosis.com", out));
  TEST_ASSERT_EQUAL_STRING("https://sarj.rotosis.com", out);
  TEST_ASSERT_TRUE(normalize_server_url("https://sarj.rotosis.com//", out));
  TEST_ASSERT_EQUAL_STRING("https://sarj.rotosis.com", out);
  TEST_ASSERT_TRUE(normalize_server_url("https://example.com:8443/api.ashx", out));
  TEST_ASSERT_EQUAL_STRING("https://example.com:8443/api.ashx", out);
  TEST_ASSERT_FALSE(normalize_server_url("http://sarj.rotosis.com", out));
  TEST_ASSERT_FALSE(normalize_server_url("https://", out));
  TEST_ASSERT_FALSE(normalize_server_url("https://sarj rotosis.com", out));
  TEST_ASSERT_FALSE(normalize_server_url("https://a.com:99999", out));
  TEST_ASSERT_FALSE(normalize_server_url("https://a.com/\"x", out));
  TEST_ASSERT_FALSE(normalize_server_url("https://a.com?x=1", out));
}

static void test_build_json_basic_and_ack() {
  Telemetry t;
  t.state = 'C';
  t.ia = 15.8f;
  t.pW = 3630.0f;
  t.eKWh = 1234.567f;
  t.tSec = 5321;
  Ack ack;
  char buf[512];
  int n = build_poll_json(buf, sizeof(buf), "4571A4", kTestSecret, t, "1.1.63", ack);
  TEST_ASSERT_GREATER_THAN(0, n);
  TEST_ASSERT_EQUAL_INT((int)strlen(buf), n);

  StaticJsonDocument<768> d;
  TEST_ASSERT_TRUE(deserializeJson(d, buf) == DeserializationError::Ok);
  TEST_ASSERT_EQUAL_STRING("4571A4", d["code"]);
  TEST_ASSERT_EQUAL_STRING(kTestSecret, d["secret"]);
  TEST_ASSERT_EQUAL_STRING("C", d["state"]);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 15.8, d["ia"].as<float>());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1234.567, d["eKWh"].as<float>());
  TEST_ASSERT_EQUAL_INT(5321, d["tSec"].as<int>());
  TEST_ASSERT_EQUAL_STRING("1.1.63", d["fw"]);
  TEST_ASSERT_TRUE(d["ackId"].isNull());
  TEST_ASSERT_FALSE(d["paused"].as<bool>());
  TEST_ASSERT_TRUE(d["pauseReason"].isNull());
  TEST_ASSERT_TRUE(d["resumeInSec"].isNull());
  TEST_ASSERT_TRUE(d["sched"]["nextChangeInSec"].isNull());  // bilinmiyor -> null (sunucu 0..604800)

  ack.has = true;
  ack.id = 41;
  ack.ok = false;
  n = build_poll_json(buf, sizeof(buf), "4571A4", kTestSecret, t, "1.1.63", ack);
  TEST_ASSERT_GREATER_THAN(0, n);
  d.clear();
  TEST_ASSERT_TRUE(deserializeJson(d, buf) == DeserializationError::Ok);
  TEST_ASSERT_EQUAL_INT(41, d["ackId"].as<int>());
  TEST_ASSERT_TRUE(d["ackOk"].is<bool>());
  TEST_ASSERT_FALSE(d["ackOk"].as<bool>());
}

static void test_build_json_paused_and_errors() {
  Telemetry t;
  t.state = 'B';
  t.paused = true;
  t.resumeInSec = 3600;
  t.schedOn = true;
  t.schedBlocked = true;
  t.schedTimeOk = true;
  t.schedNextChangeInSec = 3600;
  Ack ack;
  char buf[512];
  TEST_ASSERT_GREATER_THAN(0, build_poll_json(buf, sizeof(buf), "IST01", kTestSecret, t, "1.1.63", ack));
  StaticJsonDocument<768> d;
  TEST_ASSERT_TRUE(deserializeJson(d, buf) == DeserializationError::Ok);
  TEST_ASSERT_TRUE(d["paused"].as<bool>());
  TEST_ASSERT_EQUAL_STRING("schedule", d["pauseReason"]);
  TEST_ASSERT_EQUAL_INT(3600, d["resumeInSec"].as<int>());
  TEST_ASSERT_TRUE(d["sched"]["on"].as<bool>());
  TEST_ASSERT_TRUE(d["sched"]["blocked"].as<bool>());
  TEST_ASSERT_EQUAL_INT(3600, d["sched"]["nextChangeInSec"].as<int>());

  // Gecersiz kod/anahtar -> -1, tampon bos (anahtar sizmaz)
  TEST_ASSERT_EQUAL_INT(-1, build_poll_json(buf, sizeof(buf), "IST\"01", kTestSecret, t, "1", ack));
  TEST_ASSERT_EQUAL_STRING("", buf);
  TEST_ASSERT_EQUAL_INT(-1, build_poll_json(buf, sizeof(buf), "IST01", "", t, "1", ack));
  // Kucuk tampon -> -1 ve bos
  char small[40];
  TEST_ASSERT_EQUAL_INT(-1, build_poll_json(small, sizeof(small), "IST01", kTestSecret, t, "1", ack));
  TEST_ASSERT_EQUAL_STRING("", small);
  // Durum harfi normalize, fw kacisi
  t.state = 'X';
  TEST_ASSERT_GREATER_THAN(0, build_poll_json(buf, sizeof(buf), "IST01", kTestSecret, t, "1.0\"x", ack));
  d.clear();
  TEST_ASSERT_TRUE(deserializeJson(d, buf) == DeserializationError::Ok);
  TEST_ASSERT_EQUAL_STRING("F", d["state"]);
  TEST_ASSERT_EQUAL_STRING("?", d["fw"]);
}

static void test_parse_response() {
  PollResponse r;
  const char* a = "{\"ok\":true,\"t\":1791289200,\"cmd\":null}";
  TEST_ASSERT_TRUE(parse_poll_response(a, strlen(a), r));
  TEST_ASSERT_TRUE(r.ok);
  TEST_ASSERT_FALSE(r.hasCmd);
  TEST_ASSERT_TRUE(r.t == 1791289200LL);

  const char* b = "{\"ok\":true,\"t\":1791289203,\"cmd\":{\"id\":41,\"type\":\"start\",\"payload\":null}}";
  TEST_ASSERT_TRUE(parse_poll_response(b, strlen(b), r));
  TEST_ASSERT_TRUE(r.hasCmd);
  TEST_ASSERT_TRUE(r.cmdId == 41);
  TEST_ASSERT_TRUE(r.type == CmdType::Start);

  const char* c = "{\"ok\":true,\"t\":1,\"cmd\":{\"id\":42,\"type\":\"stop\",\"payload\":null}}";
  TEST_ASSERT_TRUE(parse_poll_response(c, strlen(c), r));
  TEST_ASSERT_TRUE(r.type == CmdType::Stop);
  const char* e = "{\"ok\":true,\"t\":1,\"cmd\":{\"id\":43,\"type\":\"set_limit\",\"payload\":{\"a\":16}}}";
  TEST_ASSERT_TRUE(parse_poll_response(e, strlen(e), r));
  TEST_ASSERT_TRUE(r.type == CmdType::SetLimit);
  const char* f = "{\"ok\":true,\"t\":1,\"cmd\":{\"id\":44,\"type\":\"reboot\"}}";
  TEST_ASSERT_TRUE(parse_poll_response(f, strlen(f), r));
  TEST_ASSERT_TRUE(r.type == CmdType::Unknown);

  const char* g = "{\"ok\":false,\"error\":\"unauthorized\"}";
  TEST_ASSERT_TRUE(parse_poll_response(g, strlen(g), r));
  TEST_ASSERT_FALSE(r.ok);

  const char* bad1 = "{\"ok\":tru";
  TEST_ASSERT_FALSE(parse_poll_response(bad1, strlen(bad1), r));
  const char* bad2 = "{\"ok\":true,\"cmd\":{\"type\":\"start\"}}";  // id yok
  TEST_ASSERT_FALSE(parse_poll_response(bad2, strlen(bad2), r));
  const char* bad3 = "{\"ok\":true,\"cmd\":{\"id\":0,\"type\":\"start\"}}";
  TEST_ASSERT_FALSE(parse_poll_response(bad3, strlen(bad3), r));
  const char* bad4 = "[1,2]";
  TEST_ASSERT_FALSE(parse_poll_response(bad4, strlen(bad4), r));
  TEST_ASSERT_FALSE(parse_poll_response("", 0, r));
}

static void test_command_dedupe() {
  CommandTracker tr;
  TEST_ASSERT_FALSE(tr.pending_ack().has);
  TEST_ASSERT_TRUE(tr.on_command(41) == CommandTracker::Decision::Apply);
  // Uygulanirken ayni komut tekrar gelirse yeniden uygulanmaz
  TEST_ASSERT_TRUE(tr.on_command(41) == CommandTracker::Decision::InFlight);
  TEST_ASSERT_TRUE(tr.on_command(42) == CommandTracker::Decision::Ignore);
  TEST_ASSERT_FALSE(tr.pending_ack().has);  // sonuc yokken ack gonderilmez
  tr.on_applied(41, true);
  Ack a = tr.pending_ack();
  TEST_ASSERT_TRUE(a.has);
  TEST_ASSERT_TRUE(a.id == 41);
  TEST_ASSERT_TRUE(a.ok);
  // Ack kayboldu, sunucu yeniden gonderdi: uygulanmaz, ayni sonucla yeniden onaylanir
  TEST_ASSERT_TRUE(tr.on_command(41) == CommandTracker::Decision::ReAck);
  tr.on_ack_delivered(41);
  TEST_ASSERT_FALSE(tr.pending_ack().has);
  TEST_ASSERT_TRUE(tr.on_command(41) == CommandTracker::Decision::ReAck);
  TEST_ASSERT_TRUE(tr.pending_ack().has);
  tr.on_ack_delivered(41);
  // Yeni komut
  TEST_ASSERT_TRUE(tr.on_command(42) == CommandTracker::Decision::Apply);
  tr.on_applied(42, false);
  TEST_ASSERT_FALSE(tr.pending_ack().ok);
  // Baska id icin ack teslimi bekleyen ack'i silmez
  tr.on_ack_delivered(41);
  TEST_ASSERT_TRUE(tr.pending_ack().has);
  // Yanlis id ile on_applied etkisiz
  tr.on_applied(99, true);
  TEST_ASSERT_TRUE(tr.last_id() == 42);
}

static void test_command_dedupe_after_reboot() {
  CommandTracker tr;
  tr.restore(true, 77, true);  // NVS'den
  TEST_ASSERT_TRUE(tr.on_command(77) == CommandTracker::Decision::ReAck);
  TEST_ASSERT_TRUE(tr.pending_ack().ok);
  TEST_ASSERT_TRUE(tr.on_command(78) == CommandTracker::Decision::Apply);
  CommandTracker empty;
  empty.restore(false, 77, true);
  TEST_ASSERT_TRUE(empty.on_command(77) == CommandTracker::Decision::Apply);
}

static void test_safe_gate_off_keeps_behavior() {
  SafeGate g;
  const char states[] = {'A', 'B', 'C', 'D', 'E', 'F'};
  for (char s : states) TEST_ASSERT_TRUE(g.update(false, s, false));
  TEST_ASSERT_TRUE(g.on_start('B', false));   // kapali modda start: kabul, izin RAM'de tutulmaz
  TEST_ASSERT_FALSE(g.authorized());
  TEST_ASSERT_FALSE(g.on_start('A', false));  // arac yok
  TEST_ASSERT_TRUE(g.update(false, 'C', true));
}

static void test_safe_gate_require_start() {
  SafeGate g;
  // Arac takildi ama bulut start yok -> izin yok
  TEST_ASSERT_FALSE(g.update(true, 'B', false));
  // Arac yokken start reddedilir
  TEST_ASSERT_FALSE(g.on_start('A', true));
  TEST_ASSERT_FALSE(g.update(true, 'A', false));
  // B'de start -> izin
  TEST_ASSERT_TRUE(g.on_start('B', true));
  TEST_ASSERT_TRUE(g.update(true, 'B', false));
  TEST_ASSERT_TRUE(g.update(true, 'C', false));
  // Baglanti kopmasi izni kaldirmaz (artik parametre yok): uzun sure C'de kalir
  for (int i = 0; i < 1000; ++i) TEST_ASSERT_TRUE(g.update(true, 'C', false));
  // Arac sarji bitirdi C->B: izin surer (arac yeniden isterse devam)
  TEST_ASSERT_TRUE(g.update(true, 'B', false));
  // E/F mevcut koruma ile PWM'i keser; izin korunur
  TEST_ASSERT_TRUE(g.update(true, 'E', false));
  // Arac ayrildi -> izin kalkar, yeniden takilinca yeni start gerekir
  TEST_ASSERT_FALSE(g.update(true, 'A', false));
  TEST_ASSERT_FALSE(g.update(true, 'B', false));
  // stop komutu izni kaldirir
  TEST_ASSERT_TRUE(g.on_start('C', true));
  TEST_ASSERT_TRUE(g.update(true, 'C', false));
  TEST_ASSERT_TRUE(g.on_stop());
  TEST_ASSERT_FALSE(g.update(true, 'C', false));
  // Yerel panel STOP izni kaldirir
  TEST_ASSERT_TRUE(g.on_start('B', true));
  TEST_ASSERT_FALSE(g.update(true, 'B', true));
  TEST_ASSERT_FALSE(g.update(true, 'B', false));
  // Mod kapatilinca eski izin kalmaz
  TEST_ASSERT_TRUE(g.on_start('B', true));
  TEST_ASSERT_TRUE(g.update(false, 'B', false));
  TEST_ASSERT_FALSE(g.update(true, 'B', false));
}

// ---------------- Planli sarj ----------------
using namespace evse_sched;

// 2026-10-05 Pazartesi 00:00 yerel (UTC+3) = 2026-10-04 21:00 UTC
static const int64_t kMon0 = 1791147600LL;
static int64_t at(int day, int h, int m, int s = 0) { return kMon0 + day * 86400LL + h * 3600LL + m * 60LL + s; }

static Schedule peak(uint8_t days = kAllDays) {
  Schedule s = default_schedule();
  s.enabled = true;
  s.w[0].days = days;
  return s;
}

static void test_sched_local_parts() {
  uint8_t wd = 9;
  uint16_t mod = 0;
  uint32_t sow = 0;
  local_parts(kMon0, &wd, &mod, &sow);
  TEST_ASSERT_EQUAL_UINT8(0, wd);
  TEST_ASSERT_EQUAL_UINT16(0, mod);
  TEST_ASSERT_EQUAL_UINT32(0, sow);
  local_parts(at(6, 23, 59), &wd, &mod, &sow);  // Pazar 23:59
  TEST_ASSERT_EQUAL_UINT8(6, wd);
  TEST_ASSERT_EQUAL_UINT16(23 * 60 + 59, mod);
  local_parts(at(7, 0, 0), &wd, &mod, nullptr);  // sonraki Pazartesi
  TEST_ASSERT_EQUAL_UINT8(0, wd);
}

static void test_sched_default_off() {
  Schedule s = default_schedule();
  TEST_ASSERT_FALSE(s.enabled);
  TEST_ASSERT_EQUAL_UINT8(1, s.count);
  TEST_ASSERT_EQUAL_UINT16(17 * 60, s.w[0].startMin);
  TEST_ASSERT_EQUAL_UINT16(22 * 60, s.w[0].endMin);
  Eval e = evaluate(s, at(0, 18, 0));
  TEST_ASSERT_FALSE(e.blocked);   // kapaliyken kisitlama yok (mevcut davranis)
  TEST_ASSERT_TRUE(e.timeValid);
  TEST_ASSERT_EQUAL_INT32(-1, e.nextChangeInSec);
}

static void test_sched_peak_window() {
  Schedule s = peak();
  Eval e = evaluate(s, at(0, 16, 59));
  TEST_ASSERT_FALSE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(60, e.nextChangeInSec);
  TEST_ASSERT_EQUAL_INT16(17 * 60, e.untilMin);
  e = evaluate(s, at(0, 16, 59, 30));
  TEST_ASSERT_EQUAL_INT32(30, e.nextChangeInSec);
  e = evaluate(s, at(0, 17, 0));
  TEST_ASSERT_TRUE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(5 * 3600, e.nextChangeInSec);
  TEST_ASSERT_EQUAL_INT16(22 * 60, e.untilMin);
  e = evaluate(s, at(2, 21, 59, 59));
  TEST_ASSERT_TRUE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(1, e.nextChangeInSec);
  e = evaluate(s, at(0, 22, 0));
  TEST_ASSERT_FALSE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(19 * 3600, e.nextChangeInSec);  // ertesi gun 17:00
  // Pazar 22:00 -> Pazartesi 17:00 (hafta tasmasi)
  e = evaluate(s, at(6, 22, 0));
  TEST_ASSERT_FALSE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(19 * 3600, e.nextChangeInSec);
}

static void test_sched_overnight_and_days() {
  Schedule s;
  s.enabled = true;
  s.count = 1;
  s.w[0].startMin = 23 * 60;
  s.w[0].endMin = 6 * 60;
  s.w[0].days = 0x01;  // yalnizca Pazartesi (Pzt 23:00 -> Sal 06:00)
  TEST_ASSERT_TRUE(validate(s, nullptr));
  TEST_ASSERT_FALSE(evaluate(s, at(0, 22, 59)).blocked);
  TEST_ASSERT_TRUE(evaluate(s, at(0, 23, 30)).blocked);
  Eval e = evaluate(s, at(1, 5, 59));
  TEST_ASSERT_TRUE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(60, e.nextChangeInSec);
  TEST_ASSERT_EQUAL_INT16(6 * 60, e.untilMin);
  TEST_ASSERT_FALSE(evaluate(s, at(1, 6, 0)).blocked);
  TEST_ASSERT_FALSE(evaluate(s, at(1, 23, 30)).blocked);   // Sali secili degil
  TEST_ASSERT_FALSE(evaluate(s, at(0, 2, 0)).blocked);     // Pazartesi sabahi (Pazar secili degil)
  // Pazar gecesi -> Pazartesi sabahi (hafta tasmasi)
  s.w[0].days = 0x40;
  TEST_ASSERT_TRUE(evaluate(s, at(0, 2, 0)).blocked);
  TEST_ASSERT_TRUE(evaluate(s, at(6, 23, 0)).blocked);
  TEST_ASSERT_FALSE(evaluate(s, at(6, 22, 59)).blocked);
  // Yalnizca hafta ici puant: Cumartesi serbest, sonraki degisim Pazartesi 17:00
  Schedule w = peak(0x1F);
  Eval c = evaluate(w, at(5, 18, 0));
  TEST_ASSERT_FALSE(c.blocked);
  TEST_ASSERT_EQUAL_INT32(47 * 3600, c.nextChangeInSec);
  TEST_ASSERT_TRUE(evaluate(w, at(4, 18, 0)).blocked);  // Cuma
}

static void test_sched_adjacent_windows() {
  Schedule s;
  s.enabled = true;
  s.count = 2;
  s.w[0] = {17 * 60, 19 * 60, kAllDays};
  s.w[1] = {19 * 60, 22 * 60, kAllDays};
  Eval e = evaluate(s, at(0, 18, 0));
  TEST_ASSERT_TRUE(e.blocked);
  TEST_ASSERT_EQUAL_INT32(4 * 3600, e.nextChangeInSec);  // 19:00 sinirinda degisim yok
  TEST_ASSERT_EQUAL_INT16(22 * 60, e.untilMin);
  // Her gun tum gun (00:00-00:00 gecersiz; 00:00-23:59 + 23:59-00:00 tum hafta) -> degisim yok
  Schedule full;
  full.enabled = true;
  full.count = 2;
  full.w[0] = {0, 23 * 60 + 59, kAllDays};
  full.w[1] = {23 * 60 + 59, 0, kAllDays};
  Eval f = evaluate(full, at(3, 12, 0));
  TEST_ASSERT_TRUE(f.blocked);
  TEST_ASSERT_EQUAL_INT32(-1, f.nextChangeInSec);
}

static void test_sched_time_invalid() {
  Schedule s = peak();
  Eval e = evaluate(s, 1000);  // NTP gelmemis (1970)
  TEST_ASSERT_FALSE(e.timeValid);
  TEST_ASSERT_FALSE(e.blocked);  // saat yoksa kisitlama UYGULANMAZ
  TEST_ASSERT_EQUAL_INT32(-1, e.nextChangeInSec);
  TEST_ASSERT_EQUAL_INT16(-1, e.nowMin);
  e = evaluate(s, kMinValidEpoch - 1);
  TEST_ASSERT_FALSE(e.timeValid);
}

static void test_sched_validate_and_hhmm() {
  const char* err = nullptr;
  Schedule s = peak();
  TEST_ASSERT_TRUE(validate(s, &err));
  s.w[0].endMin = s.w[0].startMin;
  TEST_ASSERT_FALSE(validate(s, &err));
  TEST_ASSERT_TRUE(strlen(err) > 0);
  s = peak();
  s.w[0].days = 0;
  TEST_ASSERT_FALSE(validate(s, &err));
  s = peak();
  s.count = 4;
  TEST_ASSERT_FALSE(validate(s, &err));
  s = peak();
  s.w[0].startMin = 1440;
  TEST_ASSERT_FALSE(validate(s, &err));
  s = peak();
  s.count = 0;
  TEST_ASSERT_FALSE(validate(s, &err));  // acik ama aralik yok
  s.enabled = false;
  TEST_ASSERT_TRUE(validate(s, &err));

  uint16_t m = 0;
  TEST_ASSERT_TRUE(parse_hhmm("17:00", &m));
  TEST_ASSERT_EQUAL_UINT16(1020, m);
  TEST_ASSERT_TRUE(parse_hhmm("7:05", &m));
  TEST_ASSERT_EQUAL_UINT16(425, m);
  TEST_ASSERT_TRUE(parse_hhmm("23:59", &m));
  TEST_ASSERT_FALSE(parse_hhmm("24:00", &m));
  TEST_ASSERT_FALSE(parse_hhmm("12:60", &m));
  TEST_ASSERT_FALSE(parse_hhmm("12:5", &m));
  TEST_ASSERT_FALSE(parse_hhmm("1200", &m));
  TEST_ASSERT_FALSE(parse_hhmm("12:00x", &m));
  TEST_ASSERT_FALSE(parse_hhmm("", &m));
  char b[6];
  format_hhmm(1320, b);
  TEST_ASSERT_EQUAL_STRING("22:00", b);
  format_hhmm(5, b);
  TEST_ASSERT_EQUAL_STRING("00:05", b);
}

static void test_sched_blob_roundtrip() {
  Schedule s;
  s.enabled = true;
  s.count = 3;
  s.w[0] = {17 * 60, 22 * 60, kAllDays};
  s.w[1] = {23 * 60, 6 * 60, 0x1F};
  s.w[2] = {12 * 60 + 30, 13 * 60, 0x60};
  uint8_t blob[kBlobLen];
  serialize(s, blob);
  Schedule r;
  TEST_ASSERT_TRUE(deserialize(blob, sizeof(blob), &r));
  TEST_ASSERT_TRUE(r.enabled);
  TEST_ASSERT_EQUAL_UINT8(3, r.count);
  TEST_ASSERT_EQUAL_UINT16(23 * 60, r.w[1].startMin);
  TEST_ASSERT_EQUAL_UINT16(6 * 60, r.w[1].endMin);
  TEST_ASSERT_EQUAL_UINT8(0x60, r.w[2].days);
  blob[0] = 99;  // bilinmeyen surum
  TEST_ASSERT_FALSE(deserialize(blob, sizeof(blob), &r));
  serialize(s, blob);
  TEST_ASSERT_FALSE(deserialize(blob, sizeof(blob) - 1, &r));
  blob[2] = 7;  // count bozuk
  TEST_ASSERT_FALSE(deserialize(blob, sizeof(blob), &r));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_backoff);
  RUN_TEST(test_config_validation);
  RUN_TEST(test_build_json_basic_and_ack);
  RUN_TEST(test_build_json_paused_and_errors);
  RUN_TEST(test_parse_response);
  RUN_TEST(test_command_dedupe);
  RUN_TEST(test_command_dedupe_after_reboot);
  RUN_TEST(test_safe_gate_off_keeps_behavior);
  RUN_TEST(test_safe_gate_require_start);
  RUN_TEST(test_sched_local_parts);
  RUN_TEST(test_sched_default_off);
  RUN_TEST(test_sched_peak_window);
  RUN_TEST(test_sched_overnight_and_days);
  RUN_TEST(test_sched_adjacent_windows);
  RUN_TEST(test_sched_time_invalid);
  RUN_TEST(test_sched_validate_and_hhmm);
  RUN_TEST(test_sched_blob_roundtrip);
  return UNITY_END();
}
