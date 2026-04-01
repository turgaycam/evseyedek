#pragma once

// Web, Wi-Fi, OTA ve JSON endpoint modulunun giris noktalari.
// Yeni endpoint eklenince implementasyon web_ui.cpp icinde, cagri ise genelde main.cpp'de degil burada kalir.

void web_init();
void web_loop();
bool web_ready_for_ota_validation();
