#include "schedule_core.h"

#include <string.h>

namespace evse_sched {

namespace {
constexpr uint32_t kWeekMin = 7u * 1440u;
constexpr uint32_t kWeekSec = kWeekMin * 60u;
constexpr uint8_t kBlobVersion = 1;
}  // namespace

Schedule default_schedule() {
  Schedule s;
  s.enabled = false;
  s.count = 1;
  s.w[0].startMin = 17 * 60;
  s.w[0].endMin = 22 * 60;
  s.w[0].days = kAllDays;
  return s;
}

bool validate(const Schedule& s, const char** err) {
  const char* dummy;
  if (err == nullptr) err = &dummy;
  *err = "";
  if (s.count > kMaxWindows) { *err = "En cok 3 aralik olabilir"; return false; }
  if (s.enabled && s.count == 0) { *err = "Acik zamanlayici icin en az 1 aralik gerekli"; return false; }
  for (uint8_t i = 0; i < s.count; ++i) {
    const Window& w = s.w[i];
    if (w.startMin >= 1440 || w.endMin >= 1440) { *err = "Saat 00:00-23:59 olmali"; return false; }
    if (w.startMin == w.endMin) { *err = "Baslangic ve bitis ayni olamaz"; return false; }
    if ((w.days & kAllDays) == 0 || (w.days & ~kAllDays) != 0) { *err = "En az bir gun secilmeli"; return false; }
  }
  return true;
}

void local_parts(int64_t utcEpoch, uint8_t* weekday, uint16_t* minuteOfDay, uint32_t* secOfWeek) {
  int64_t local = utcEpoch + kTzOffsetSec;
  int64_t days = local / 86400;
  int64_t secOfDay = local % 86400;
  if (secOfDay < 0) { secOfDay += 86400; days -= 1; }
  // 1970-01-01 Persembe; Pazartesi=0 icin +3
  int64_t wd = (days + 3) % 7;
  if (wd < 0) wd += 7;
  if (weekday) *weekday = (uint8_t)wd;
  if (minuteOfDay) *minuteOfDay = (uint16_t)(secOfDay / 60);
  if (secOfWeek) *secOfWeek = (uint32_t)(wd * 86400 + secOfDay);
}

bool blocked_at_week_minute(const Schedule& s, uint32_t weekMinute) {
  weekMinute %= kWeekMin;
  for (uint8_t i = 0; i < s.count && i < kMaxWindows; ++i) {
    const Window& w = s.w[i];
    if (w.startMin == w.endMin) continue;
    uint32_t dur = (w.endMin > w.startMin) ? (uint32_t)(w.endMin - w.startMin)
                                           : (uint32_t)(1440 - w.startMin + w.endMin);
    for (uint8_t d = 0; d < 7; ++d) {
      if (!(w.days & (1u << d))) continue;
      uint32_t start = d * 1440u + w.startMin;
      uint32_t off = (weekMinute + kWeekMin - start) % kWeekMin;  // pazar->pazartesi tasmasi dahil
      if (off < dur) return true;
    }
  }
  return false;
}

Eval evaluate(const Schedule& s, int64_t utcEpoch) {
  Eval e;
  e.enabled = s.enabled;
  e.timeValid = utcEpoch >= kMinValidEpoch;
  uint8_t wd = 0;
  uint16_t mod = 0;
  uint32_t sow = 0;
  if (e.timeValid) {
    local_parts(utcEpoch, &wd, &mod, &sow);
    e.nowMin = (int16_t)mod;
  }
  if (!s.enabled || !e.timeValid || s.count == 0) return e;

  uint32_t nowWeekMin = sow / 60u;
  e.blocked = blocked_at_week_minute(s, nowWeekMin);

  // Aday sinirlar: her aralik/gun icin baslangic ve bitis dakikalari (en cok 3*7*2).
  uint32_t best = 0;
  bool found = false;
  for (uint8_t i = 0; i < s.count && i < kMaxWindows; ++i) {
    const Window& w = s.w[i];
    if (w.startMin == w.endMin) continue;
    uint32_t dur = (w.endMin > w.startMin) ? (uint32_t)(w.endMin - w.startMin)
                                           : (uint32_t)(1440 - w.startMin + w.endMin);
    for (uint8_t d = 0; d < 7; ++d) {
      if (!(w.days & (1u << d))) continue;
      uint32_t cands[2] = {(d * 1440u + w.startMin) % kWeekMin, (d * 1440u + w.startMin + dur) % kWeekMin};
      for (uint32_t c : cands) {
        uint32_t delta = (c * 60u + kWeekSec - sow) % kWeekSec;
        if (delta == 0) continue;  // simdiki an; durum zaten bunu yansitiyor
        if (found && delta >= best) continue;
        // Bu sinirda durum gercekten degisiyor mu?
        bool st = blocked_at_week_minute(s, c);
        if (st != e.blocked) {
          best = delta;
          found = true;
        }
      }
    }
  }
  if (found) {
    // Durum yalnizca aday sinirlarda degisir; durumu farkli olan en yakin aday = ilk degisim
    // (bitisik/ortusen araliklarda degismeyen sinirlar atlanir).
    e.nextChangeInSec = (int32_t)best;
    uint32_t at = (sow + best) % kWeekSec;
    e.untilMin = (int16_t)((at / 60u) % 1440u);
  }
  return e;
}

bool parse_hhmm(const char* s, uint16_t* outMin) {
  if (s == nullptr) return false;
  // "H:MM" veya "HH:MM"
  int h = 0, m = 0, i = 0, digits = 0;
  while (s[i] >= '0' && s[i] <= '9' && digits < 2) { h = h * 10 + (s[i] - '0'); ++i; ++digits; }
  if (digits == 0 || s[i] != ':') return false;
  ++i;
  if (!(s[i] >= '0' && s[i] <= '9' && s[i + 1] >= '0' && s[i + 1] <= '9') || s[i + 2] != '\0') return false;
  m = (s[i] - '0') * 10 + (s[i + 1] - '0');
  if (h > 23 || m > 59) return false;
  if (outMin) *outMin = (uint16_t)(h * 60 + m);
  return true;
}

void format_hhmm(uint16_t minute, char out[6]) {
  minute %= 1440;
  out[0] = (char)('0' + (minute / 60) / 10);
  out[1] = (char)('0' + (minute / 60) % 10);
  out[2] = ':';
  out[3] = (char)('0' + (minute % 60) / 10);
  out[4] = (char)('0' + (minute % 60) % 10);
  out[5] = '\0';
}

void serialize(const Schedule& s, uint8_t out[kBlobLen]) {
  memset(out, 0, kBlobLen);
  out[0] = kBlobVersion;
  out[1] = s.enabled ? 1 : 0;
  out[2] = s.count > kMaxWindows ? kMaxWindows : s.count;
  for (uint8_t i = 0; i < kMaxWindows; ++i) {
    uint8_t* p = out + 4 + i * 6;
    p[0] = (uint8_t)(s.w[i].startMin & 0xFF);
    p[1] = (uint8_t)(s.w[i].startMin >> 8);
    p[2] = (uint8_t)(s.w[i].endMin & 0xFF);
    p[3] = (uint8_t)(s.w[i].endMin >> 8);
    p[4] = s.w[i].days;
  }
}

bool deserialize(const uint8_t* in, size_t len, Schedule* out) {
  if (in == nullptr || out == nullptr || len != kBlobLen || in[0] != kBlobVersion) return false;
  Schedule s;
  s.enabled = in[1] != 0;
  s.count = in[2];
  if (s.count > kMaxWindows) return false;
  for (uint8_t i = 0; i < kMaxWindows; ++i) {
    const uint8_t* p = in + 4 + i * 6;
    s.w[i].startMin = (uint16_t)(p[0] | (p[1] << 8));
    s.w[i].endMin = (uint16_t)(p[2] | (p[3] << 8));
    s.w[i].days = p[4];
  }
  if (!validate(s, nullptr)) return false;
  *out = s;
  return true;
}

}  // namespace evse_sched
