#pragma once

// Planli sarj (puant korumasi) - cihaz tarafi. Mantik: schedule_core.h (host'ta birim testli).
// - Ayar NVS'de ("evsesched"); VARSAYILAN KAPALI (tek aralik 17:00-22:00, her gun).
// - Saat NTP'den (UTC+3 sabit). Saat gecersizse kisitlama uygulanmaz.
// - Bulut baglantisindan bagimsiz calisir; NTP'yi de bu modul baslatir.

#include <stddef.h>
#include "schedule_core.h"

void sched_init();

// Ana dongude her turda cagrilir (saniyede bir degerlendirir). Donus: su an yasakli mi.
bool sched_loop();

evse_sched::Eval sched_eval();
evse_sched::Schedule sched_get();
// Dogrular, NVS'ye yazar. Hatada false ve err.
bool sched_set(const evse_sched::Schedule& s, const char** err);

// {"on":..,"timeOk":..,"blocked":..,"nextChangeInSec":..,"until":"HH:MM","now":"HH:MM"[,"windows":[...]]}
void sched_status_json(char* out, size_t outLen, bool withWindows);
