// secrets.example.h — ORNEK dosya. Kendi secrets.h'inizi bu sablonla olusturun.
// Gercek secrets.h .gitignore'da oldugu icin GitHub'a gitmez.
#pragma once

// Wi-Fi AP
#define EVSE_AP_SSID "EVSE"
#define EVSE_AP_PASSWORD "sifrenizi_yazin"

// Admin panel
#define EVSE_ADMIN_USER "admin"
#define EVSE_ADMIN_PASSWORD "sifrenizi_yazin"

// mDNS / OTA hostname
#define EVSE_HOSTNAME "evse_istasyon"
#define EVSE_OTA_HOSTNAME "evse_istasyon"
#define EVSE_OTA_PASSWORD ""

// Telegram bot
#define TELEGRAM_BOT_TOKEN "bot_tokeninizi_yazin"
#define TELEGRAM_CHAT_ID "chat_id_nizi_yazin"

// Bilinen Wi-Fi aglari (5 adede kadar)
#define EVSE_WIFI_1_LOC "Ev"
#define EVSE_WIFI_1_SSID "WiFi_adiniz"
#define EVSE_WIFI_1_PASS "WiFi_sifreniz"

#define EVSE_WIFI_2_LOC "Ofis"
#define EVSE_WIFI_2_SSID "WiFi_adiniz"
#define EVSE_WIFI_2_PASS "WiFi_sifreniz"

#define EVSE_WIFI_3_LOC ""
#define EVSE_WIFI_3_SSID ""
#define EVSE_WIFI_3_PASS ""

#define EVSE_WIFI_4_LOC ""
#define EVSE_WIFI_4_SSID ""
#define EVSE_WIFI_4_PASS ""

#define EVSE_WIFI_5_LOC ""
#define EVSE_WIFI_5_SSID ""
#define EVSE_WIFI_5_PASS ""