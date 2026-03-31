#pragma once
#include <Arduino.h>

// Varsayilan firmware versiyonu build bayragi ile gelmezse burada tanimlanir.
// PlatformIO'da build_flags icine -DCURRENT_VERSION="1.0.0" ekleyerek override edebilirsin.
#ifndef CURRENT_VERSION
#define CURRENT_VERSION "1.0.0"
#endif

namespace OTA_Manager {

// OTA manifesti; GitHub'daki version.json icin URL ver.
void begin(const char* manifestUrl,
           uint32_t checkIntervalMs = 60UL * 60UL * 1000UL, // 1 saat
           const char* sha1Fingerprint = "");

// Her loop'ta cagirilir: rollback dogrulama ve periyodik update kontrolu.
void loop();

// Manuel tetikleme (ornek: web komutundan). Wi-Fi bagli ise hemen kontrol eder.
void triggerCheckNow();
void deferPeriodicChecks(uint32_t ms);

// Son bilinen durum ve versiyonlar.
const char* currentVersion();
bool lastUpdateSucceeded();
const char* lastRemoteVersion();
const char* lastStatusText();
const char* lastErrorText();
uint32_t lastCheckAgeMs();

}  // namespace OTA_Manager

