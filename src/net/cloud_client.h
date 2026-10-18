#pragma once

// Sarj sunucusu (varsayilan https://sarj.rotosis.com) bulut istemcisi - cihaz tarafi.
// - Istasyon kodu, sunucu adresi ve gizli anahtar DERLEMEYE GOMULMEZ; NVS'de ("evsecloud") tutulur ve
//   yonetim panelinden (POST /api/cloud_cfg, Bearer) girilir. Anahtar hicbir yanitta/gunlukte geri donmez.
// - Anahtar yoksa ya da bulut kapaliysa istemci pasiftir; cihaz eskisi gibi calisir.
// - Ayri FreeRTOS gorevi (cekirdek 0) her ~2,5 sn HTTPS ile /api/device/poll yapar; ana donguyu bloklamaz.
// - Komutlar (start/stop) ana dongude cloud_loop() icinde uygulanir; role/PWM'e gorevden dokunulmaz.
// Mantik cekirdegi: cloud_core.h (host'ta birim testli).

#include <Arduino.h>
#include <stddef.h>

// setup() icinde bir kez (web_init'ten sonra).
void cloud_init();

// Ana dongude, her olcum turunda PWM karari verilmeden once cagrilir (bulut pasif olsa da).
// Telemetriyi gorev icin yayinlar, bekleyen bulut komutunu uygular, guvenli mod kapisini ve
// planli sarj (schedBlocked) beklemesini isletir; izin kalkinca roleyi guvenle birakir.
// Donus: false ise bu turda PWM kapatilmali ve role latch takibi atlanmali (sarja izin yok).
// requireCloudStart ve zamanlayici kapaliyken her zaman true doner (mevcut davranis).
bool cloud_loop(const String& stableState, float ia, float ib, float ic,
                float powerW, float energyKWh, uint32_t chargeSeconds, bool schedBlocked);

// Bulut etkin mi (acik + gecerli kod/adres + anahtar ayarli)?
bool cloud_active();
bool cloud_require_start();

// Panelden ayar degisikligi. Verilmeyen alan (nullptr / -1) degismez.
// on/requireStart: -1 degismez, 0 kapali, 1 acik. code "" -> varsayilan (MAC son 6 hane).
// clearSecret=true anahtari siler. Hepsi once dogrulanir; hata varsa hicbiri uygulanmaz.
struct CloudCfgChange {
  int on = -1;
  int requireStart = -1;
  const char* code = nullptr;
  const char* url = nullptr;
  const char* secret = nullptr;
  bool clearSecret = false;
};
// Donus: true basarili; false ise err kisa aciklama (ASCII).
bool cloud_apply_config(const CloudCfgChange& ch, const char** err);

// /status ve /api/cloud_cfg icin durum nesnesi (gizli anahtar ASLA icermez; yalnizca secretSet).
void cloud_status_json(char* out, size_t outLen);
