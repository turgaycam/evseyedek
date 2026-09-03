#include "app_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace {
constexpr uint8_t kLogLines = 40;
constexpr uint8_t kLogLineLen = 96;

char s_lines[kLogLines][kLogLineLen];
uint8_t s_head = 0;
uint8_t s_count = 0;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void pushLine(const char* text)
{
  portENTER_CRITICAL(&s_mux);
  uint8_t idx;
  if (s_count < kLogLines) {
    idx = (s_head + s_count) % kLogLines;
    s_count++;
  } else {
    idx = s_head;
    s_head = (s_head + 1) % kLogLines;
  }
  strncpy(s_lines[idx], text, kLogLineLen - 1);
  s_lines[idx][kLogLineLen - 1] = '\0';
  portEXIT_CRITICAL(&s_mux);
}
}  // namespace

void app_logf(const char* fmt, ...)
{
  char body[kLogLineLen];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(body, sizeof(body), fmt, ap);
  va_end(ap);

  char line[kLogLineLen];
  snprintf(line, sizeof(line), "%lus %s", (unsigned long)(millis() / 1000UL), body);
  Serial.println(line);
  pushLine(line);
}

void app_log_dump(char* out, size_t outLen, uint8_t maxLines)
{
  if (out == nullptr || outLen == 0) return;
  out[0] = '\0';
  if (maxLines == 0) return;

  portENTER_CRITICAL(&s_mux);
  uint8_t count = s_count;
  uint8_t head = s_head;
  if (count > maxLines) {
    head = (head + (uint8_t)(count - maxLines)) % kLogLines;
    count = maxLines;
  }
  size_t used = 0;
  for (uint8_t i = 0; i < count && used + 2 < outLen; i++) {
    const char* src = s_lines[(head + i) % kLogLines];
    size_t n = strlen(src);
    if (used + n + 2 >= outLen) break;
    memcpy(out + used, src, n);
    used += n;
    out[used++] = '\n';
    out[used] = '\0';
  }
  portEXIT_CRITICAL(&s_mux);
}
