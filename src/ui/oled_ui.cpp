#include "oled_ui.h"
#include "app_pins.h"
#include <U8g2lib.h>
#include <Wire.h>

// OLED arayuz modulu.
// Bu dosyanin ic yapisi:
// 1) label / format helper'lari
// 2) sag ust state strip
// 3) orta ana panel
// 4) alt metrik satiri

// 1.3" 128x64 SH1106 I2C
// Reset pini yoksa U8X8_PIN_NONE kullanilir.
static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE
);

static bool oledAvailable = false;
static uint8_t oledAddress = 0;
static const float kPhaseOnThresholdA = 0.90f;
static const uint32_t kStateBlinkMs = 800;
static const uint32_t kErrorBlinkMs = 500;

// OLED adresini 0x3C / 0x3D arasindan bularak calismaya baslar.
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
  if (stateStable == "D") return "SARJ";
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

// OLED'de buyuk gosterilecek akim degeri icin aktif fazlarin ortalamasini kullanir.
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

// Alt satirdaki sureyi kisa tutmak icin HH:MM formatina cevirir.
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

// Sag ustteki mini state gostergesi.
static void draw_state_box(int x, const char* text, bool active, bool blinkOn)
{
  const int y = 1;
  const int w = 10;
  const int h = 10;
  u8g2.setFont(u8g2_font_5x7_tf);

  if (active && blinkOn) {
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

// Sag ustte sadece aktif state yanip soner.
static void draw_state_strip(const String& stateStable)
{
  const bool blinkOn = ((millis() / kStateBlinkMs) % 2U) == 0U;

  draw_state_box(88, "A", stateStable == "A", blinkOn);
  draw_state_box(98, "B", stateStable == "B", blinkOn);
  draw_state_box(108, "C", stateStable == "C", blinkOn);
  draw_state_box(118, "D", stateStable == "D", blinkOn);
}

// Baslik solda state yazisi, sagda mini state strip'ten olusur.
static void draw_header(const String& stateStable)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 9, ui_state_label(stateStable));
  draw_state_strip(stateStable);
  u8g2.drawHLine(0, 12, 128);
}

// Orta panel: bekleme / sarj / hata senaryolarinin ana mesajini cizer.
static void draw_main_panel(const String& stateStable, float ia, float ib, float ic)
{
  const bool errorState = is_error_state(stateStable);
  const bool charging = (stateStable == "C" || stateStable == "D");

  if (errorState) {
    const bool blinkOn = ((millis() / kErrorBlinkMs) % 2U) == 0U;
    if (blinkOn) {
      u8g2.drawRBox(0, 16, 128, 27, 3);
      u8g2.setDrawColor(0);
    } else {
      u8g2.drawRFrame(0, 16, 128, 27, 3);
    }

    u8g2.setFont(u8g2_font_7x13B_tf);
    draw_centered_text(30, "HATA");
    u8g2.setFont(u8g2_font_6x10_tf);
    draw_centered_text(41, "Sarj durdu");
    u8g2.setDrawColor(1);
    return;
  }

  u8g2.drawRFrame(0, 16, 128, 27, 3);

  if (charging) {
    char currentText[12];
    float currentA = display_current(ia, ib, ic);
    snprintf(currentText, sizeof(currentText), "%.1f", currentA);

    u8g2.setFont(u8g2_font_5x7_tf);
    draw_centered_text(23, "SARJ OLUYOR");

    u8g2.setFont(u8g2_font_logisoso18_tn);
    int valueWidth = u8g2.getStrWidth(currentText);
    int valueX = (128 - valueWidth - 12) / 2;
    if (valueX < 4) valueX = 4;
    u8g2.drawStr(valueX, 42, currentText);

    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(valueX + valueWidth + 3, 40, "A");
  } else {
    u8g2.setFont(u8g2_font_7x13B_tf);
    draw_centered_text(31, ui_idle_title(stateStable));

    u8g2.setFont(u8g2_font_6x10_tf);
    draw_centered_text(41, ui_idle_subtitle(stateStable));
  }
}

// Alt satir kisa teknik ozet icindir.
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

  // Bu fonksiyon yalnizca cizim yapar; karar mantigi main.cpp tarafinda uretilir.
  u8g2.clearBuffer();
  draw_header(stateStable);
  draw_main_panel(stateStable, ia, ib, ic);
  draw_metrics(ia, ib, ic, powerW, energyKwh, chargeSeconds);

  u8g2.sendBuffer();
}
