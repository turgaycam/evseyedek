#ifndef OLED_UI_H
#define OLED_UI_H

#include <Arduino.h>

// OLED gorunum modulu.
// Layout degisiklikleri dogrudan oled_ui.cpp icinde yapilir.

// Ekranı ve I2C'yi başlatır
void oled_init();

// Ekrani son durum verileriyle gunceller.
// stateStable: pilot state'i
// ia/ib/ic: faz akimlari
// powerW / energyKwh / chargeSeconds: main.cpp'deki hesaplanan ozet veriler
void oled_draw(const String& stateStable,
               float ia,
               float ib,
               float ic,
               float powerW,
               float energyKwh,
               uint32_t chargeSeconds,
               bool relayOn,
               bool staConnected,
               bool cableConnected);

#endif
