#pragma once
#include <Arduino.h>

// Role modulu hem klasik role cikisini hem de latch set/reset cikislarini yonetir.
// Donanim degisikliginde app_pins.h ve relay.cpp birlikte kontrol edilmelidir.

void relay_init();
void relay_set(bool on);
bool relay_get();
void relay_force_off_now();

// Röle AUTO yönetimi
void relay_update_auto(const String& stableState, bool pwmEnabled, int pwmDutyPercent);

// AUTO kontrol
void relay_set_auto_enabled(bool en);
bool relay_is_auto_enabled();

// Röle geçişlerini yavaşlat (ms)
void relay_set_min_switch_ms(uint32_t ms);

// SET/RESET latch pulse'larini stable state gecislerine gore tetikle
void relay_handle_state_pulse(const String& stableState);
