#include "current_sensor.h"
#include "app_pins.h"
#include "EmonLib.h"
#include <Preferences.h>
#include <Arduino.h>
#include <math.h>

// Akim sensor modulu.
// Yeni CT karti veya farkli kalibrasyon gelirse ilk mudahale edilecek yer burasidir.

static EnergyMonitor emonA;
static EnergyMonitor emonB;
static EnergyMonitor emonC;
static Preferences s_calPrefs;
static bool s_calPrefsReady = false;
static constexpr float kDefaultCalA = 12.0f; // fine-tune: 23.17 / 23.7
static constexpr float kDefaultCalB = 12.0f; // fine-tune: 25.17 / 25.6
static constexpr float kDefaultCalC = 12.0f; // fine-tune: 24.28 / 24.7
static constexpr float kDefaultLowRangeMaxA = 10.0f;
static constexpr float kDefaultMidRangeMaxA = 30.0f;
static constexpr float kDefaultLowRangeOffsetA = 0.0f;
static constexpr float kDefaultMidRangeOffsetA = 1.0f;
static constexpr float kMinIcal = 1.0f;
static constexpr float kMaxIcal = 80.0f;
static constexpr float kMinOff = -10.0f;
static constexpr float kMaxOff = 10.0f;
static constexpr float kMinRangeMaxA = 1.0f;
static constexpr float kMaxRangeMaxA = 80.0f;
static constexpr uint32_t kCalMagic = 0x43414C31; // "CAL1"
static constexpr uint16_t kCalVersion = 2;

static float s_calA = kDefaultCalA;
static float s_calB = kDefaultCalB;
static float s_calC = kDefaultCalC;
static float s_offA = 0.0f;
static float s_offB = 0.0f;
static float s_offC = 0.0f;
static bool s_enA = true;
static bool s_enB = true;
static bool s_enC = true;
static float s_lowRangeMaxA = kDefaultLowRangeMaxA;
static float s_midRangeMaxA = kDefaultMidRangeMaxA;
static float s_lowRangeOffsetA = kDefaultLowRangeOffsetA;
static float s_midRangeOffsetA = kDefaultMidRangeOffsetA;
static float last_filtered_a = 0.0f;
static float last_filtered_b = 0.0f;
static float last_filtered_c = 0.0f;
static float last_irms_a = 0.0f;
static float last_irms_b = 0.0f;
static float last_irms_c = 0.0f;
static float noise_floor_a = 0.0f;
static float noise_floor_b = 0.0f;
static float noise_floor_c = 0.0f;
static bool phase_active_a = false;
static bool phase_active_b = false;
static bool phase_active_c = false;

struct CalStore {
    uint32_t magic;
    uint16_t version;
    float calA;
    float calB;
    float calC;
    float offA;
    float offB;
    float offC;
    uint8_t enA;
    uint8_t enB;
    uint8_t enC;
    uint8_t reserved;
};

static void apply_calibration() {
    emonA.current(CURRENT_SENSOR_PIN_A, s_calA);
    emonB.current(CURRENT_SENSOR_PIN_B, s_calB);
    emonC.current(CURRENT_SENSOR_PIN_C, s_calC);
}

static void save_calibration_nvs() {
    if (!s_calPrefsReady) return;
    CalStore data{};
    data.magic = kCalMagic;
    data.version = kCalVersion;
    data.calA = s_calA;
    data.calB = s_calB;
    data.calC = s_calC;
    data.offA = s_offA;
    data.offB = s_offB;
    data.offC = s_offC;
    data.enA = s_enA ? 1 : 0;
    data.enB = s_enB ? 1 : 0;
    data.enC = s_enC ? 1 : 0;
    s_calPrefs.putBytes("iCal", &data, sizeof(data));
}

static void save_range_profile_nvs() {
    if (!s_calPrefsReady) return;
    s_calPrefs.putFloat("rngLowMax", s_lowRangeMaxA);
    s_calPrefs.putFloat("rngMidMax", s_midRangeMaxA);
    s_calPrefs.putFloat("rngLowOff", s_lowRangeOffsetA);
    s_calPrefs.putFloat("rngMidOff", s_midRangeOffsetA);
}

static bool valid_cal(const CalStore& d) {
    return (d.magic == kCalMagic) &&
           (d.version == kCalVersion) &&
           isfinite(d.calA) && isfinite(d.calB) && isfinite(d.calC) &&
           isfinite(d.offA) && isfinite(d.offB) && isfinite(d.offC) &&
           d.calA >= kMinIcal && d.calA <= kMaxIcal &&
           d.calB >= kMinIcal && d.calB <= kMaxIcal &&
           d.calC >= kMinIcal && d.calC <= kMaxIcal &&
           d.offA >= kMinOff && d.offA <= kMaxOff &&
           d.offB >= kMinOff && d.offB <= kMaxOff &&
           d.offC >= kMinOff && d.offC <= kMaxOff &&
           d.enA <= 1 && d.enB <= 1 && d.enC <= 1;
}

static void load_calibration_nvs() {
    if (!s_calPrefsReady) return;
    CalStore data{};
    size_t got = s_calPrefs.getBytes("iCal", &data, sizeof(data));
    if (got == sizeof(data) && valid_cal(data)) {
        s_calA = data.calA;
        s_calB = data.calB;
        s_calC = data.calC;
        s_offA = data.offA;
        s_offB = data.offB;
        s_offC = data.offC;
        s_enA = data.enA != 0;
        s_enB = data.enB != 0;
        s_enC = data.enC != 0;
    } else {
        // Eski/hatali kaydi varsayilan (12/12/12) ile guncelle.
        save_calibration_nvs();
    }
}

static void load_range_profile_nvs() {
    if (!s_calPrefsReady) return;

    float lowMax = s_calPrefs.getFloat("rngLowMax", kDefaultLowRangeMaxA);
    float midMax = s_calPrefs.getFloat("rngMidMax", kDefaultMidRangeMaxA);
    float lowOff = s_calPrefs.getFloat("rngLowOff", kDefaultLowRangeOffsetA);
    float midOff = s_calPrefs.getFloat("rngMidOff", kDefaultMidRangeOffsetA);

    if (!isfinite(lowMax) || !isfinite(midMax) ||
        !isfinite(lowOff) || !isfinite(midOff)) {
        save_range_profile_nvs();
        return;
    }

    s_lowRangeMaxA = constrain(lowMax, kMinRangeMaxA, kMaxRangeMaxA);
    s_midRangeMaxA = constrain(midMax, s_lowRangeMaxA, kMaxRangeMaxA);
    s_lowRangeOffsetA = constrain(lowOff, kMinOff, kMaxOff);
    s_midRangeOffsetA = constrain(midOff, kMinOff, kMaxOff);
}

static float range_profile_offset(float currentA) {
    if (currentA <= 0.0f) return 0.0f;
    if (currentA <= s_lowRangeMaxA) return s_lowRangeOffsetA;
    if (currentA <= s_midRangeMaxA) return s_midRangeOffsetA;
    return 0.0f;
}

// Raw Irms degerini gurultu tabani, hysteresis ve EMA ile daha okunabilir hale getirir.
static float apply_phase_filter(float raw, float* noise_floor, bool* phase_active, float last_value) {
    // Dusuk seviyede yavas noise-floor kalibrasyonu.
    if (raw < 2.2f) {
        *noise_floor = (*noise_floor * 0.985f) + (raw * 0.015f);
    }

    float corrected = raw - (*noise_floor + 0.15f);
    if (corrected < 0.0f) corrected = 0.0f;

    // Histerezis: kucuk dalgalanmalarda akimi "var" gostermesin.
    const float onTh = 1.00f;
    const float offTh = 0.45f;
    if (!(*phase_active) && corrected >= onTh) *phase_active = true;
    if ((*phase_active) && corrected <= offTh) *phase_active = false;
    if (!(*phase_active)) corrected = 0.0f;

    // Stabil gorunum icin EMA. Dususte daha yavas davranarak ekrani sakin tut.
    const float alphaUp = 0.18f;
    const float alphaDown = 0.09f;
    float alpha = (corrected >= last_value) ? alphaUp : alphaDown;
    float filtered = last_value + ((corrected - last_value) * alpha);
    if (filtered < 0.08f) filtered = 0.0f;
    return filtered;
}

void current_sensor_init() {
    // ADC Ayarlari
    analogReadResolution(12);       // ESP32-S3 icin 12-bit (0-4095)
    analogSetAttenuation(ADC_11db); // 3.3V giris araligi icin

    pinMode(CURRENT_SENSOR_PIN_A, INPUT);
    pinMode(CURRENT_SENSOR_PIN_B, INPUT);
    pinMode(CURRENT_SENSOR_PIN_C, INPUT);

    s_calPrefsReady = s_calPrefs.begin("evsecal", false);
    load_calibration_nvs();
    load_range_profile_nvs();

    // Her fazin kalibrasyon katsayisi burada ayarlanir.
    apply_calibration();

    // Baslangicta anlik gurultu seviyesini yakala.
    noise_floor_a = emonA.calcIrms(180);
    noise_floor_b = emonB.calcIrms(180);
    noise_floor_c = emonC.calcIrms(180);
}

void current_sensor_loop() {
    // Her turda tek faz okunur; web arayuzunu bloklamamak icin dagitilmis olcum kullanilir.
    static uint32_t last_read = 0;
    static uint8_t phase_index = 0;
    const uint32_t now = millis();

    if (now - last_read < 400) return;
    last_read = now;

    const int sampleCount = 180;
    if (phase_index == 0) {
        if (s_enA) {
            float raw = emonA.calcIrms(sampleCount);
            float filtered = apply_phase_filter(raw, &noise_floor_a, &phase_active_a, last_filtered_a);
            last_filtered_a = filtered;
            last_irms_a = filtered + range_profile_offset(filtered) + s_offA;
            if (last_irms_a < 0.0f) last_irms_a = 0.0f;
        } else {
            last_filtered_a = 0.0f;
            last_irms_a = 0.0f;
            phase_active_a = false;
        }
    } else if (phase_index == 1) {
        if (s_enB) {
            float raw = emonB.calcIrms(sampleCount);
            float filtered = apply_phase_filter(raw, &noise_floor_b, &phase_active_b, last_filtered_b);
            last_filtered_b = filtered;
            last_irms_b = filtered + range_profile_offset(filtered) + s_offB;
            if (last_irms_b < 0.0f) last_irms_b = 0.0f;
        } else {
            last_filtered_b = 0.0f;
            last_irms_b = 0.0f;
            phase_active_b = false;
        }
    } else {
        if (s_enC) {
            float raw = emonC.calcIrms(sampleCount);
            float filtered = apply_phase_filter(raw, &noise_floor_c, &phase_active_c, last_filtered_c);
            last_filtered_c = filtered;
            last_irms_c = filtered + range_profile_offset(filtered) + s_offC;
            if (last_irms_c < 0.0f) last_irms_c = 0.0f;
        } else {
            last_filtered_c = 0.0f;
            last_irms_c = 0.0f;
            phase_active_c = false;
        }
    }

    yield();
    phase_index = (phase_index + 1) % 3;
}

float current_sensor_get_irms_a() { return last_irms_a; }
float current_sensor_get_irms_b() { return last_irms_b; }
float current_sensor_get_irms_c() { return last_irms_c; }
float current_sensor_get_irms_total() { return last_irms_a + last_irms_b + last_irms_c; }

void current_sensor_set_calibration(float calA, float calB, float calC,
                                    float offA, float offB, float offC) {
    s_calA = constrain(calA, kMinIcal, kMaxIcal);
    s_calB = constrain(calB, kMinIcal, kMaxIcal);
    s_calC = constrain(calC, kMinIcal, kMaxIcal);
    s_offA = constrain(offA, kMinOff, kMaxOff);
    s_offB = constrain(offB, kMinOff, kMaxOff);
    s_offC = constrain(offC, kMinOff, kMaxOff);
    apply_calibration();
    save_calibration_nvs();
}

void current_sensor_get_calibration(float* calA, float* calB, float* calC,
                                    float* offA, float* offB, float* offC) {
    if (calA) *calA = s_calA;
    if (calB) *calB = s_calB;
    if (calC) *calC = s_calC;
    if (offA) *offA = s_offA;
    if (offB) *offB = s_offB;
    if (offC) *offC = s_offC;
}

void current_sensor_set_range_profile(float lowMaxA,
                                      float midMaxA,
                                      float lowOffsetA,
                                      float midOffsetA) {
    s_lowRangeMaxA = constrain(lowMaxA, kMinRangeMaxA, kMaxRangeMaxA);
    s_midRangeMaxA = constrain(midMaxA, s_lowRangeMaxA, kMaxRangeMaxA);
    s_lowRangeOffsetA = constrain(lowOffsetA, kMinOff, kMaxOff);
    s_midRangeOffsetA = constrain(midOffsetA, kMinOff, kMaxOff);
    save_range_profile_nvs();
}

void current_sensor_get_range_profile(float* lowMaxA,
                                      float* midMaxA,
                                      float* lowOffsetA,
                                      float* midOffsetA) {
    if (lowMaxA) *lowMaxA = s_lowRangeMaxA;
    if (midMaxA) *midMaxA = s_midRangeMaxA;
    if (lowOffsetA) *lowOffsetA = s_lowRangeOffsetA;
    if (midOffsetA) *midOffsetA = s_midRangeOffsetA;
}

void current_sensor_set_enabled(bool enA, bool enB, bool enC) {
    s_enA = enA;
    s_enB = enB;
    s_enC = enC;
    save_calibration_nvs();
}

void current_sensor_get_enabled(bool* enA, bool* enB, bool* enC) {
    if (enA) *enA = s_enA;
    if (enB) *enB = s_enB;
    if (enC) *enC = s_enC;
}
