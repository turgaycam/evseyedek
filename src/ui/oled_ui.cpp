#include "oled_ui.h"
#include "app_pins.h"
#include <U8g2lib.h>
#include <Wire.h>

// 1.3" 128x64 SH1106 I2C
// Reset pini yoksa U8X8_PIN_NONE kullanilir.
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE
);

static bool oledAvailable = false;
static uint8_t oledAddress = 0;
static const float kPhaseOnThresholdA = 0.90f;

static bool probe_oled_addr(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static bool is_error_state(const String& stateStable)
{
  return (stateStable == "E" || stateStable == "F");
}

static const char* ui_state_label(const String& stateStable)
{
  if (stateStable == "A") return "ARAC YOK";
  if (stateStable == "B") return "HAZIR";
  if (stateStable == "C") return "SARJ";
  if (stateStable == "D") return "HAVALANDIR";
  if (stateStable == "E" || stateStable == "F") return "HATA";
  return "DURUM";
}

static const char* ui_idle_title(const String& stateStable)
{
  if (stateStable == "A") return "BEKLE";
  if (stateStable == "B") return "HAZIR";
  if (stateStable == "E" || stateStable == "F") return "HATA";
  return "EVSE";
}

static const char* ui_idle_subtitle(const String& stateStable)
{
  if (stateStable == "A") return "Kablo bekleniyor";
  if (stateStable == "B") return "Arac bagli";
  if (stateStable == "E" || stateStable == "F") return "Sarj durduruldu";
  return "Durum izleniyor";
}

static const char* phase_label(float ia, float ib, float ic)
{
  if (ib > kPhaseOnThresholdA || ic > kPhaseOnThresholdA) return "3F";
  if (ia > kPhaseOnThresholdA) return "1F";
  return "--";
}

static float display_current(float ia, float ib, float ic)
{
  float sum = 0.0f;
  uint8_t count = 0;

  if (ia > kPhaseOnThresholdA) {
    sum += ia;
    count++;
  }
  if (ib > kPhaseOnThresholdA) {
    sum += ib;
    count++;
  }
  if (ic > kPhaseOnThresholdA) {
    sum += ic;
    count++;
  }

  if (count == 0) return 0.0f;
  return sum / (float)count;
}

static void format_duration_hhmm(uint32_t chargeSeconds, char* out, size_t outSize)
{
  uint32_t hh = chargeSeconds / 3600;
  uint32_t mm = (chargeSeconds % 3600) / 60;
  if (hh > 99) hh = 99;
  snprintf(out, outSize, "%02lu:%02lu", (unsigned long)hh, (unsigned long)mm);
}

static void draw_centered_text(int y, const char* text)
{
  int x = (128 - u8g2.getStrWidth(text)) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, y, text);
}

static void draw_status_badge(int x, const char* text, bool active)
{
  const int y = 1;
  const int w = 10;
  const int h = 10;
  u8g2.setFont(u8g2_font_5x7_tf);

  if (active) {
    u8g2.drawRBox(x, y, w, h, 2);
    u8g2.setDrawColor(0);
  } else {
    u8g2.drawRFrame(x, y, w, h, 2);
  }

  int tx = x + (w - u8g2.getStrWidth(text)) / 2;
  if (tx < x + 1) tx = x + 1;
  u8g2.drawStr(tx, 9, text);
  u8g2.setDrawColor(1);
}

static void draw_header(const String& stateStable,
                        bool staConnected,
                        bool cableConnected,
                        bool relayOn)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 9, ui_state_label(stateStable));
  draw_status_badge(94, "W", staConnected);
  draw_status_badge(106, "K", cableConnected);
  draw_status_badge(118, "R", relayOn);
  u8g2.drawHLine(0, 12, 128);
}

static void draw_main_panel(const String& stateStable, float ia, float ib, float ic)
{
  const bool charging = (stateStable == "C" || stateStable == "D");
  u8g2.drawRFrame(0, 16, 128, 27, 3);

  if (charging) {
    char currentText[12];
    float currentA = display_current(ia, ib, ic);
    snprintf(currentText, sizeof(currentText), "%.1f", currentA);

    u8g2.setFont(u8g2_font_6x10_tf);
    draw_centered_text(26, (stateStable == "D") ? "HAVALANDIRMA" : "SARJ DEVAM");

    u8g2.setFont(u8g2_font_logisoso18_tn);
    int valueWidth = u8g2.getStrWidth(currentText);
    int valueX = (128 - valueWidth - 12) / 2;
    if (valueX < 4) valueX = 4;
    u8g2.drawStr(valueX, 41, currentText);

    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(valueX + valueWidth + 3, 39, "A");
  } else {
    u8g2.setFont(u8g2_font_7x13B_tf);
    draw_centered_text(31, ui_idle_title(stateStable));

    u8g2.setFont(u8g2_font_6x10_tf);
    draw_centered_text(41, ui_idle_subtitle(stateStable));
  }
}

static void draw_metrics(float ia,
                         float ib,
                         float ic,
                         float powerW,
                         float energyKwh,
                         uint32_t chargeSeconds)
{
  char line1[28];
  char line2[28];
  char timeText[8];

  format_duration_hhmm(chargeSeconds, timeText, sizeof(timeText));

  u8g2.setFont(u8g2_font_5x7_tf);
  snprintf(line1, sizeof(line1), "Guc:%.1fkW Faz:%s", powerW / 1000.0f, phase_label(ia, ib, ic));
  snprintf(line2, sizeof(line2), "E:%.2fkWh T:%s", energyKwh, timeText);

  u8g2.drawStr(2, 52, line1);
  u8g2.drawStr(2, 62, line2);
}

void oled_init()
{
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  if (probe_oled_addr(0x3C)) {
    oledAddress = 0x3C;
    oledAvailable = true;
  } else if (probe_oled_addr(0x3D)) {
    oledAddress = 0x3D;
    oledAvailable = true;
  } else {
    oledAvailable = false;
    Serial.println("OLED bulunamadi: 0x3C / 0x3D cevap vermedi.");
    return;
  }

  u8g2.setI2CAddress(oledAddress << 1);
  u8g2.begin();
  Serial.printf("OLED hazir: SH1106 @ 0x%02X\n", oledAddress);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 30, "Rotosis EVSE");
  u8g2.drawStr(10, 45, "Baslatiliyor...");
  u8g2.sendBuffer();
}

void oled_draw(const String& stateStable,
               float ia,
               float ib,
               float ic,
               float powerW,
               float energyKwh,
               uint32_t chargeSeconds,
               bool relayOn,
               bool staConnected,
               bool cableConnected)
{
  if (!oledAvailable) return;

  u8g2.clearBuffer();
  draw_header(stateStable, staConnected, cableConnected, relayOn);
  draw_main_panel(stateStable, ia, ib, ic);
  draw_metrics(ia, ib, ic, powerW, energyKwh, chargeSeconds);

  u8g2.sendBuffer();
}
