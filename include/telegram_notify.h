#pragma once
#include <Arduino.h>

// Telegram I/O ana donguyu bloke etmez.
// Giden mesajlar kuyruga yazilir, HTTP arka plan gorevinde gider.
// Komutlar: yardim / durum / bilgi / guncelle / geri al / yeniden baslat / isim
void telegram_loop(const String& pilotState, bool wifiConnected);
