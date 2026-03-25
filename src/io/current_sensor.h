#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

// 3 faz akim olcum modulu.
// Kalibrasyon ve filtre ayarlari current_sensor.cpp icindedir.

void current_sensor_init();
void current_sensor_loop();
float current_sensor_get_irms_a();
float current_sensor_get_irms_b();
float current_sensor_get_irms_c();
float current_sensor_get_irms_total();

#endif
