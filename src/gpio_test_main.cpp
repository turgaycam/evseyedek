#include <Arduino.h>

// Bu dosya ana EVSE firmware'i degildir.
// [env:esp32-s3-gpio-test] secilince yalnizca LED / GPIO testi icin derlenir.

constexpr uint8_t kLedPins[] = {8, 17, 18};
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
  Serial.println("Pins: GPIO8, GPIO17, GPIO18");
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
