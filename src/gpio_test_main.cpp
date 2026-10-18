#include <Arduino.h>
#include "app_pins.h"

// Bu dosya ana EVSE firmware'i degildir.
// [env:esp32-s3-gpio-test] secilince yalnizca LED / GPIO testi icin derlenir.
// Pinler include/app_pins.h ile ayni olmak zorundadir.

constexpr uint8_t kLedPins[] = {STATE_LED_PIN, WIFI_LED_PIN, ERROR_LED_PIN};
constexpr uint8_t kLedOn = HIGH;
constexpr uint8_t kLedOff = LOW;
constexpr uint32_t kOnMs = 500;
constexpr uint32_t kOffMs = 350;
constexpr uint32_t kGapMs = 800;

void setLed(uint8_t pin, bool on) {
  digitalWrite(pin, on ? kLedOn : kLedOff);
}

void allOff() {
  for (uint8_t pin : kLedPins) {
    setLed(pin, false);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("GPIO LED TEST START");
  Serial.printf("Pins: STATE GPIO%d, WIFI GPIO%d, ERROR GPIO%d\n",
                STATE_LED_PIN, WIFI_LED_PIN, ERROR_LED_PIN);
  Serial.println("Pattern: each LED ON/OFF in sequence");

  for (uint8_t pin : kLedPins) {
    pinMode(pin, OUTPUT);
    setLed(pin, false);
  }
}

void loop() {
  for (uint8_t pin : kLedPins) {
    Serial.printf("GPIO %u -> ON\n", pin);
    setLed(pin, true);
    delay(kOnMs);

    Serial.printf("GPIO %u -> OFF\n", pin);
    setLed(pin, false);
    delay(kOffMs);
  }

  Serial.println("Cycle done.");
  delay(kGapMs);
}
