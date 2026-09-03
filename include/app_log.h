#pragma once
#include <Arduino.h>

// 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL 2
#endif

void app_logf(const char* fmt, ...);
void app_log_dump(char* out, size_t outLen, uint8_t maxLines);

#define LOGE(fmt, ...) do { if (APP_LOG_LEVEL >= 0) app_logf("[E] " fmt, ##__VA_ARGS__); } while (0)
#define LOGW(fmt, ...) do { if (APP_LOG_LEVEL >= 1) app_logf("[W] " fmt, ##__VA_ARGS__); } while (0)
#define LOGI(fmt, ...) do { if (APP_LOG_LEVEL >= 2) app_logf("[I] " fmt, ##__VA_ARGS__); } while (0)
#define LOGD(fmt, ...) do { if (APP_LOG_LEVEL >= 3) app_logf("[D] " fmt, ##__VA_ARGS__); } while (0)
