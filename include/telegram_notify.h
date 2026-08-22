#pragma once
#include <Arduino.h>

// Cihaz WiFi'ye baglandiginda bir kez Telegram'a mesaj gonderir.
// Mesaj icerigi: Kart ID, WiFi SSID, RSSI, IP, FW surumu, pilot state.
void telegram_notify_connect(const String& pilotState);

// Pilot state degistiginde (A/B/C/D/E/F) Telegram'a bildirim gonderir.
// main loop'ta telegram_notify_connect ile birlikte cagirilmali.
void telegram_notify_state_change(const String& pilotState);

// WiFi baglanti durumunu takip eder. Baglanti kopup geri geldiginde
// kesinti raporu gonderir (10 sn'den kisa dalgalanmalar bildirilmez).
// main loop'ta her turda staOk durumuyla cagirilmali.
void telegram_notify_wifi_link(bool connected);

// Telegram'dan gelen komutlari dinler ve yanitlar.
// main loop'ta cagirilmali. Polling ayri FreeRTOS task'inda (16KB stack) calisir,
// ana loop'u bloke etmez. Komut geldiginde s_cmdReady flag'i ile islenir.
// Komutlar: /help /status /info /name /update /rollback /restart
void telegram_loop(const String& pilotState);