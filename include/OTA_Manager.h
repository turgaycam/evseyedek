#pragma once
#include <Arduino.h>

// Varsayilan firmware versiyonu build bayragi ile gelmezse burada tanimlanir.
// Asil surum kaynagi version.json; bu yedek yalnizca flag yoksa kullanilir.
#ifndef CURRENT_VERSION
#define CURRENT_VERSION "1.82"
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
void triggerInstallNow();
void deferPeriodicChecks(uint32_t ms);
bool selectFactoryBootPartition();
bool selectAlternateOtaBootPartition();

// Son bilinen durum ve versiyonlar.
const char* currentVersion();
bool lastUpdateSucceeded();
const char* lastRemoteVersion();
const char* lastStatusText();
const char* lastErrorText();
uint32_t lastCheckAgeMs();
const char* runningPartitionLabel();
const char* runningImageStateLabel();

}  // namespace OTA_Manager
