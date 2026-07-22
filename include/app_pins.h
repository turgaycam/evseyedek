#pragma once
#include <Arduino.h>

// Donanim pin haritasi burada tutulur.
// Yeni karta gectiginde ilk bakman gereken dosya budur.

// OLED (I2C: Wire.begin(SDA, SCL))
#define OLED_SDA 18
#define OLED_SCL 17

// CP / kontaktor / guc elektroniigi hatlari.
#define CP_PWM_PIN       2
#define CP_ADC_PIN       1
#define RELAY_PIN        4

// 3 faz akim sensor girisleri.
// Yeni kartta akim sensor pinleri henuz kesinlesmedi; GPIO4 kontaktor oldugu icin
// gecici olarak akim okuma devre disi.
#define CURRENT_SENSORS_ENABLED 0
#define CURRENT_SENSOR_PIN_A 5
#define CURRENT_SENSOR_PIN_B 6
#define CURRENT_SENSOR_PIN_C 7

// Ayrı RGB LED (simdilik uygulama mantiginda kullanilmiyor)
#define RGB_LED_PIN      15

#define STATE_LED_PIN    13
#define WIFI_LED_PIN     12
#define ERROR_LED_PIN    14

// Geri uyumluluk (eski isim)
#define BLUE_LED_PIN     STATE_LED_PIN

// Yeni kartta latch SET/RESET yok; kontaktor GPIO4 ile surekli seviye surulur.
#define MOSFET_RESET_PIN 7
#define MOSFET_SET_PIN   16

// Kontactor surucu aktif-high: HIGH=AC, LOW=KAPAT
#define RELAY_ACTIVE_LOW 0

// Farkli role donanimlari icin cikis secimi.
// Yeni role karti geldiginde once bu blok kontrol edilir.
// 1 = klasik role cikisi kullan
// 1 = SET/RESET latch cikislarini kullan
// 1 = latch pulse'lari dogrudan stable state gecisinden tetiklenir
// 0 = latch pulse'lari role cikisi degisiminden tetiklenir
#define RELAY_USE_OUTPUT_PIN 1
#define RELAY_USE_LATCH_PINS 0
#define RELAY_LATCH_FOLLOW_STATE 0
#define RELAY_LATCH_PULSE_MS 0

// CP PWM surucu ayarlari.
#define PWM_CHANNEL      0
#define PWM_FREQ         1000
#define PWM_RESOLUTION   10

// 32A limiti icin IEC 61851'e gore yaklasik duty.
#define PWM_DUTY_32A     53
