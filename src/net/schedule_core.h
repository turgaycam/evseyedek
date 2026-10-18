#pragma once

// Planli sarj (puant korumasi) zamanlayicisinin donanimdan bagimsiz cekirdegi.
// "Yasakli saat araliklari": en cok 3 aralik, dakika cozunurlugu, gece yarisini asan aralik
// desteklenir (aralik basladigi gune aittir), gun secimi bit maskesi (bit0=Pazartesi .. bit6=Pazar).
// Saat dilimi sabit UTC+3 (Turkiye, yaz saati yok). Saat gecersizse kisitlama UYGULANMAZ.
// Host'ta birim testli (pio test -e native).

#include <stddef.h>
#include <stdint.h>

namespace evse_sched {

constexpr uint8_t kMaxWindows = 3;
constexpr uint8_t kAllDays = 0x7F;
constexpr int32_t kTzOffsetSec = 3 * 3600;          // UTC+3
constexpr int64_t kMinValidEpoch = 1767225600;      // 2026-01-01 UTC; oncesi "saat yok"
constexpr size_t kBlobLen = 4 + kMaxWindows * 6;    // NVS kaydi

struct Window {
  uint16_t startMin = 0;  // 0..1439 (yerel)
  uint16_t endMin = 0;    // 0..1439; endMin <= startMin ise ertesi gune tasar
  uint8_t days = kAllDays;
};

struct Schedule {
  bool enabled = false;
  uint8_t count = 0;
  Window w[kMaxWindows];
};

// Varsayilan: kapali, tek aralik 17:00-22:00 her gun.
Schedule default_schedule();

// Gecerlilik: count<=3, her aralikta start!=end, 0..1439, days!=0.
// Hata durumunda err kisa Turkce aciklama (ASCII) alir.
bool validate(const Schedule& s, const char** err);

struct Eval {
  bool enabled = false;
  bool timeValid = false;
  bool blocked = false;          // su an yasakli aralikta mi (zamanlayici acik ve saat gecerliyse)
  int32_t nextChangeInSec = -1;  // durum degisimine kalan sn; bilinmiyor/yok ise -1
  int16_t untilMin = -1;         // degisimin yerel saati (dakika, 0..1439); yoksa -1
  int16_t nowMin = -1;           // yerel saat (dakika); saat yoksa -1
};

// Belirli bir UTC zamaninda zamanlayiciyi degerlendir.
Eval evaluate(const Schedule& s, int64_t utcEpoch);

// Yerel haftanin dakikasi (Pazartesi 00:00 = 0) icin yasakli mi (zamanlayici acik varsayilir).
bool blocked_at_week_minute(const Schedule& s, uint32_t weekMinute);

// Yerel hafta gunu (0=Pazartesi) ve gun dakikasi.
void local_parts(int64_t utcEpoch, uint8_t* weekday, uint16_t* minuteOfDay, uint32_t* secOfWeek);

// "HH:MM" <-> dakika
bool parse_hhmm(const char* s, uint16_t* outMin);
void format_hhmm(uint16_t minute, char out[6]);

// NVS kaydi (surumlu, sabit uzunluk)
void serialize(const Schedule& s, uint8_t out[kBlobLen]);
bool deserialize(const uint8_t* in, size_t len, Schedule* out);

}  // namespace evse_sched
