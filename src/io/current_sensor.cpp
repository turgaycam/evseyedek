#include "current_sensor.h"
#include "app_pins.h"
#include "EmonLib.h"
#include <Arduino.h>

// Akim sensor modulu.
// Yeni CT karti veya farkli kalibrasyon gelirse ilk mudahale edilecek yer burasidir.

static EnergyMonitor emonA;
static EnergyMonitor emonB;
static EnergyMonitor emonC;
static float last_irms_a = 0.0f;
static float last_irms_b = 0.0f;
static float last_irms_c = 0.0f;
static float noise_floor_a = 0.0f;
static float noise_floor_b = 0.0f;
static float noise_floor_c = 0.0f;
static bool phase_active_a = false;
static bool phase_active_b = false;
static bool phase_active_c = false;

// Raw Irms degerini gurultu tabani, hysteresis ve EMA ile daha okunabilir hale getirir.
static float apply_phase_filter(float raw, float* noise_floor, bool* phase_active, float last_value) {
    // Dusuk seviyede yavas noise-floor kalibrasyonu.
    if (raw < 2.0f) {
        *noise_floor = (*noise_floor * 0.98f) + (raw * 0.02f);
    }

    float corrected = raw - (*noise_floor + 0.15f);
    if (corrected < 0.0f) corrected = 0.0f;

    // Histerezis: kucuk dalgalanmalarda akimi "var" gostermesin.
    const float onTh = 0.90f;
    const float offTh = 0.35f;
    if (!(*phase_active) && corrected >= onTh) *phase_active = true;
    if ((*phase_active) && corrected <= offTh) *phase_active = false;
    if (!(*phase_active)) corrected = 0.0f;

    // Stabil gorunum icin EMA.
    const float alpha = 0.30f;
    return (last_value * (1.0f - alpha)) + (corrected * alpha);
}

void current_sensor_init() {
    // ADC Ayarlari
    analogReadResolution(12);       // ESP32-S3 icin 12-bit (0-4095)
    analogSetAttenuation(ADC_11db); // 3.3V giris araligi icin

    pinMode(CURRENT_SENSOR_PIN_A, INPUT);
    pinMode(CURRENT_SENSOR_PIN_B, INPUT);
    pinMode(CURRENT_SENSOR_PIN_C, INPUT);

    // Her fazin kalibrasyon katsayisi burada ayarlanir.
    // Pens ampermetre ile karsilastirip bu degerler duzeltilir.
    emonA.current(CURRENT_SENSOR_PIN_A, 15.01f); // fine-tune: 23.17 / 23.7
    emonB.current(CURRENT_SENSOR_PIN_B, 15.47f); // fine-tune: 25.17 / 25.6
    emonC.current(CURRENT_SENSOR_PIN_C, 27.69f); // fine-tune: 24.28 / 24.7

    // Baslangicta anlik gurultu seviyesini yakala.
    noise_floor_a = emonA.calcIrms(300);
    noise_floor_b = emonB.calcIrms(300);
    noise_floor_c = emonC.calcIrms(300);
}

void current_sensor_loop() {
    // Her turda tek faz okunur; web arayuzunu bloklamamak icin dagitilmis olcum kullanilir.
    static uint32_t last_read = 0;
    static uint8_t phase_index = 0;
    const uint32_t now = millis();

    if (now - last_read < 250) return;
    last_read = now;

    const int sampleCount = 500;
    if (phase_index == 0) {
        float raw = emonA.calcIrms(sampleCount);
        last_irms_a = apply_phase_filter(raw, &noise_floor_a, &phase_active_a, last_irms_a);
    } else if (phase_index == 1) {
        float raw = emonB.calcIrms(sampleCount);
        last_irms_b = apply_phase_filter(raw, &noise_floor_b, &phase_active_b, last_irms_b);
    } else {
        float raw = emonC.calcIrms(sampleCount);
        last_irms_c = apply_phase_filter(raw, &noise_floor_c, &phase_active_c, last_irms_c);
    }

    phase_index = (phase_index + 1) % 3;
}

float current_sensor_get_irms_a() { return last_irms_a; }
float current_sensor_get_irms_b() { return last_irms_b; }
float current_sensor_get_irms_c() { return last_irms_c; }
float current_sensor_get_irms_total() { return last_irms_a + last_irms_b + last_irms_c; }
