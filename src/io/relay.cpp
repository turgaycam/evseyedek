#include "relay.h"
#include "app_pins.h"
#include <Arduino.h>

// Role kontrol modulu.
// Web panelinden ayarlanan on/off gecikmeleri burada uygulanir.

// Yeni kartta kontaktor surekli seviye ile surulur; gecikme/pulse yok.
uint32_t relayOnDelayMs  = 0;
uint32_t relayOffDelayMs = 0;

static bool relayOn = false;
static bool relayAutoEnabled = true;

static uint32_t minSwitchMs = 0;
static uint32_t lastSwitchMs = 0;

// Zamanlayıcı için yardımcı değişkenler
static uint32_t stateChangeStartTime = 0;
static bool waitingForChange = false;
static bool pendingTargetState = false;

#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
static bool lastLatchSetState = false;
static bool latchStateReady = false;
static bool latchWaitingForPulse = false;
static bool pendingLatchSetState = false;
static uint32_t latchStateChangeStartMs = 0;
#endif

static bool isLatchSetState(const String& stableState)
{
  return stableState == "C" || stableState == "D";
}

static void writeRelayPin()
{
#if RELAY_USE_OUTPUT_PIN
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, relayOn ? LOW : HIGH);
#else
  digitalWrite(RELAY_PIN, relayOn ? HIGH : LOW);
#endif
#endif
}

static void pulseLatchPin(uint8_t pin)
{
#if RELAY_USE_LATCH_PINS
  digitalWrite(pin, HIGH);
  delay(RELAY_LATCH_PULSE_MS);
  digitalWrite(pin, LOW);
#else
  (void)pin;
#endif
}

static void applyRelayOutputs(bool pulseLatch)
{
  writeRelayPin();

#if RELAY_USE_LATCH_PINS
  if (pulseLatch && !RELAY_LATCH_FOLLOW_STATE) {
    pulseLatchPin(relayOn ? MOSFET_SET_PIN : MOSFET_RESET_PIN);
  }
#else
  (void)pulseLatch;
#endif
}

void relay_init()
{
#if RELAY_USE_OUTPUT_PIN
  pinMode(RELAY_PIN, OUTPUT);
#endif
#if RELAY_USE_LATCH_PINS
  pinMode(MOSFET_RESET_PIN, OUTPUT);
  pinMode(MOSFET_SET_PIN, OUTPUT);
  digitalWrite(MOSFET_RESET_PIN, LOW);
  digitalWrite(MOSFET_SET_PIN, LOW);
#endif
  relayOn = false;
  applyRelayOutputs(false);
}
// Role cikislarini cok sik ac/kapa yapmamak icin alt koruma.
void relay_set_min_switch_ms(uint32_t ms)
{
  minSwitchMs = ms;
}
void relay_set_auto_enabled(bool en) { relayAutoEnabled = en; }
bool relay_is_auto_enabled() { return relayAutoEnabled; }
bool relay_get() { return relayOn; }

void relay_force_off_now()
{
  relayOn = false;
  waitingForChange = false;
  pendingTargetState = false;
  writeRelayPin();

#if RELAY_USE_LATCH_PINS
  pulseLatchPin(MOSFET_RESET_PIN);
#endif

#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  latchWaitingForPulse = false;
  pendingLatchSetState = false;
  lastLatchSetState = false;
  latchStateReady = true;
  latchStateChangeStartMs = millis();
#endif

  lastSwitchMs = millis();
}

void relay_set(bool on)
{
  relayAutoEnabled = false; // Manuel müdahalede otomatiği kapat
  if (relayOn == on) return;
  relayOn = on;
  applyRelayOutputs(true);
  lastSwitchMs = millis();
}

void relay_update_auto(const String& stableState, bool pwmEnabled, int pwmDutyPercent)
{
  if (!relayAutoEnabled) return;

  (void)pwmEnabled;
  (void)pwmDutyPercent;

  // Yeni kart: GPIO4 state C/D boyunca surekli HIGH, diger state'lerde LOW.
  bool shouldBeOn = (stableState == "C" || stableState == "D");
  if (shouldBeOn == relayOn) return;

  relayOn = shouldBeOn;
  applyRelayOutputs(false);
  waitingForChange = false;
  pendingTargetState = false;
  lastSwitchMs = millis();

  Serial.printf(">>> KONTAKTOR GPIO%d: %s\n", RELAY_PIN, relayOn ? "HIGH" : "LOW");
}

void relay_handle_state_pulse(const String& stableState)
{
#if RELAY_USE_LATCH_PINS && RELAY_LATCH_FOLLOW_STATE
  // Latch donanim kullaniliyorsa SET/RESET pulse karari stable state degisimine gore verilir.
  bool latchSetState = isLatchSetState(stableState);

  if (!latchStateReady) {
    lastLatchSetState = latchSetState;
    latchStateReady = true;
    return;
  }

  // State eski haline donerse, bekleyen pulse'i iptal et.
  if (latchSetState == lastLatchSetState) {
    if (latchWaitingForPulse && pendingLatchSetState != latchSetState) {
      latchWaitingForPulse = false;
    }
    return;
  }

  if (!latchWaitingForPulse || pendingLatchSetState != latchSetState) {
    pendingLatchSetState = latchSetState;
    latchStateChangeStartMs = millis();
    latchWaitingForPulse = true;
  }

  uint32_t activeDelayMs = pendingLatchSetState ? relayOnDelayMs : relayOffDelayMs;
  if ((millis() - latchStateChangeStartMs) < activeDelayMs) return;

  pulseLatchPin(pendingLatchSetState ? MOSFET_SET_PIN : MOSFET_RESET_PIN);
  lastLatchSetState = pendingLatchSetState;
  latchWaitingForPulse = false;
#else
  (void)stableState;
#endif
}
