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

// Kalibrasyon verisi
void current_sensor_set_calibration(float calA, float calB, float calC,
                                    float offA, float offB, float offC);
void current_sensor_get_calibration(float* calA, float* calB, float* calC,
                                    float* offA, float* offB, float* offC);
void current_sensor_set_range_profile(float lowMaxA,
                                      float midMaxA,
                                      float lowOffsetA,
                                      float midOffsetA);
void current_sensor_get_range_profile(float* lowMaxA,
                                      float* midMaxA,
                                      float* lowOffsetA,
                                      float* midOffsetA);
void current_sensor_set_enabled(bool enA, bool enB, bool enC);
void current_sensor_get_enabled(bool* enA, bool* enB, bool* enC);

#endif
