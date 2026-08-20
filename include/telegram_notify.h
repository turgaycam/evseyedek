#pragma once
#include <Arduino.h>

// Cihaz WiFi'ye baglandiginda bir kez Telegram'a mesaj gonderir.
// Mesaj icerigi: Kart ID, WiFi SSID, RSSI, IP, FW surumu, pilot state.
// Hafif tutuldu: tek HTTP GET, String minimum, bayrak ile tek seferlik gonderim.
void telegram_notify_connect(const String& pilotState);