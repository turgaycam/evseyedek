#pragma once
#include <Arduino.h>

// Pilot modulu CP hattini olcer ve A/B/C/D/E state bilgisini uretir.
// State davranisini degistirmek istiyorsan asil mudahale noktasi pilot.cpp'dir.

struct PilotMeasurements {
  float adcHigh;
  float adcLow;
  float cpHigh;
  float cpLow;
  String stateRaw;
  String stateStable;
};

void pilot_init();
void pilot_apply_pwm();
void pilot_update();
PilotMeasurements pilot_get();

// PWM kontrolü (web dokunmayacak ama main kullanacak)
extern bool pwmEnabled;
extern int  pwmDutyPercent;
