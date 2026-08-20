#pragma once
#include <Arduino.h>

// Bu dosya main.cpp icindeki runtime ayarlari baska modullere acar.
// Yeni bir ayari web/pilot/relay tarafina tasiyacaksan extern olarak buraya ekle.

// Kalibrasyon / esikler (webden degisebilir)
extern float CP_DIVIDER_RATIO;

extern float TH_A_MIN;
extern float TH_B_MIN;
extern float TH_C_MIN;
extern float TH_D_MIN;
extern float TH_E_MIN;

extern float marginUp;
extern float marginDown;

extern int   stableCount;
extern int   loopIntervalMs;

// Kart kimligi (MAC tabanli)
extern String g_boardId;       // "KART-1" veya "KART-2"
extern String g_boardMac;      // MAC adresi (XX:XX:XX:XX:XX:XX)
extern String g_boardName;     // "Eski Kart" veya "Yeni Kart"
