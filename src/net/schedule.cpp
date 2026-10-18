#include "schedule.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>

using namespace evse_sched;

namespace {

constexpr const char* kPrefsNs = "evsesched";
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
Schedule s_cfg = default_schedule();
Eval s_eval;
bool s_dirty = true;
bool s_ntpStarted = false;
uint32_t s_lastEvalMs = 0;

}  // namespace

void sched_init() {
  Schedule loaded = default_schedule();
  Preferences p;
  if (p.begin(kPrefsNs, true)) {
    uint8_t blob[kBlobLen];
    size_t n = p.getBytesLength("cfg");
    if (n == kBlobLen && p.getBytes("cfg", blob, sizeof(blob)) == kBlobLen) {
      Schedule tmp;
      if (deserialize(blob, sizeof(blob), &tmp)) loaded = tmp;
    }
    p.end();
  }
  portENTER_CRITICAL(&s_mux);
  s_cfg = loaded;
  s_dirty = true;
  portEXIT_CRITICAL(&s_mux);
  Serial.printf("[SCHED] Planli sarj %s, %u aralik\n", loaded.enabled ? "ACIK" : "kapali", (unsigned)loaded.count);
}

bool sched_loop() {
  if (!s_ntpStarted && WiFi.status() == WL_CONNECTED && WiFi.localIP()[0] != 0) {
    // UTC tutulur; yerel saat (UTC+3) schedule_core icinde hesaplanir.
    configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
    s_ntpStarted = true;
  }
  uint32_t nowMs = millis();
  portENTER_CRITICAL(&s_mux);
  bool need = s_dirty || (uint32_t)(nowMs - s_lastEvalMs) >= 1000;
  Schedule cfg = s_cfg;
  bool blocked = s_eval.blocked;
  portEXIT_CRITICAL(&s_mux);
  if (!need) return blocked;

  Eval e = evaluate(cfg, (int64_t)time(nullptr));
  portENTER_CRITICAL(&s_mux);
  bool changed = (e.blocked != s_eval.blocked);
  s_eval = e;
  s_dirty = false;
  s_lastEvalMs = nowMs;
  portEXIT_CRITICAL(&s_mux);
  if (changed) {
    Serial.printf("[SCHED] Yasakli aralik %s\n", e.blocked ? "BASLADI: sarj bekletiliyor" : "bitti");
  }
  return e.blocked;
}

Eval sched_eval() {
  portENTER_CRITICAL(&s_mux);
  Eval e = s_eval;
  portEXIT_CRITICAL(&s_mux);
  return e;
}

Schedule sched_get() {
  portENTER_CRITICAL(&s_mux);
  Schedule s = s_cfg;
  portEXIT_CRITICAL(&s_mux);
  return s;
}

bool sched_set(const Schedule& s, const char** err) {
  if (!validate(s, err)) return false;
  uint8_t blob[kBlobLen];
  serialize(s, blob);
  Preferences p;
  if (!p.begin(kPrefsNs, false)) {
    if (err) *err = "NVS acilamadi";
    return false;
  }
  size_t w = p.putBytes("cfg", blob, sizeof(blob));
  p.end();
  if (w != sizeof(blob)) {
    if (err) *err = "NVS yazilamadi";
    return false;
  }
  portENTER_CRITICAL(&s_mux);
  s_cfg = s;
  s_dirty = true;
  portEXIT_CRITICAL(&s_mux);
  Serial.printf("[SCHED] Ayar kaydedildi: %s, %u aralik\n", s.enabled ? "ACIK" : "kapali", (unsigned)s.count);
  return true;
}

void sched_status_json(char* out, size_t outLen, bool withWindows) {
  if (out == nullptr || outLen == 0) return;
  Schedule cfg = sched_get();
  Eval e = sched_eval();
  char until[6] = "";
  char now[6] = "";
  if (e.untilMin >= 0) format_hhmm((uint16_t)e.untilMin, until);
  if (e.nowMin >= 0) format_hhmm((uint16_t)e.nowMin, now);
  int n = snprintf(out, outLen,
                   "{\"on\":%s,\"timeOk\":%s,\"blocked\":%s,\"nextChangeInSec\":%ld,\"until\":\"%s\",\"now\":\"%s\"",
                   cfg.enabled ? "true" : "false", e.timeValid ? "true" : "false", e.blocked ? "true" : "false",
                   (long)e.nextChangeInSec, until, now);
  if (n < 0 || (size_t)n >= outLen) { out[0] = '\0'; return; }
  if (withWindows) {
    int k = snprintf(out + n, outLen - n, ",\"windows\":[");
    if (k < 0 || (size_t)(n + k) >= outLen) { out[0] = '\0'; return; }
    n += k;
    for (uint8_t i = 0; i < cfg.count && i < kMaxWindows; ++i) {
      char s[6], en[6];
      format_hhmm(cfg.w[i].startMin, s);
      format_hhmm(cfg.w[i].endMin, en);
      k = snprintf(out + n, outLen - n, "%s{\"s\":\"%s\",\"e\":\"%s\",\"d\":%u}", i ? "," : "", s, en,
                   (unsigned)cfg.w[i].days);
      if (k < 0 || (size_t)(n + k) >= outLen) { out[0] = '\0'; return; }
      n += k;
    }
    k = snprintf(out + n, outLen - n, "]");
    if (k < 0 || (size_t)(n + k) >= outLen) { out[0] = '\0'; return; }
    n += k;
  }
  if ((size_t)(n + 2) > outLen) { out[0] = '\0'; return; }
  out[n] = '}';
  out[n + 1] = '\0';
}
