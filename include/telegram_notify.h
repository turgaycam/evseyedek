#pragma once
#include <Arduino.h>

// Cihaz WiFi'ye baglandiginda bir kez Telegram'a mesaj gonderir.
// Mesaj icerigi: Kart ID, WiFi SSID, RSSI, IP, FW surumu, pilot state.
void telegram_notify_connect(const String& pilotState);

// Telegram'dan gelen komutlari dinler ve yanitlar.
// main loop'ta cagirilmali. Her 5 sn'de bir getUpdates yapar.
// Komutlar: /help /status /info /update /rollback /restart
void telegram_loop(const String& pilotState);