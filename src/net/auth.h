#pragma once

// Yonetim paneli jeton tabanli giris (cihaz tarafi).
// - Parola NVS'de ("evseauth") tuzlu, yinelemeli SHA-256 ozeti olarak durur; duz metin saklanmaz.
// - Oturumlar RAM'de (en cok 4), 15 dk kayan sure; yeniden baslatmada hepsi duser.
// - HTTP handler'lari web_ui.cpp icindedir; bu modul yalnizca mantigi saglar.

#include <Arduino.h>
#include "auth_core.h"

enum class AuthLoginResult { Ok, BadPassword, Locked };
enum class AuthPwResult { Ok, WrongCurrent, Locked, TooShort, TooLong, SameAsCurrent, StoreFailed };

// web_init icinden bir kez cagrilir.
void auth_init();

// Basarili giriste tokenHex (33 bayt) doldurulur.
// BadPassword: info = kalan deneme; Locked: info = kalan kilit (sn).
AuthLoginResult auth_login(const char* user, const char* password, char tokenHex[evse_auth::kTokenHexLen + 1], uint32_t* info);

// "Authorization: Bearer <hex>" basligini dogrular; gecerliyse kayan sureyi yeniler.
bool auth_check_bearer(const String& authorizationHeader);
void auth_logout(const String& authorizationHeader);

// Basarida diger tum oturumlar kapanir ve yeni jeton tokenHex'e yazilir.
// Locked durumunda info = kalan kilit (sn).
AuthPwResult auth_change_password(const char* current, const char* next, char tokenHex[evse_auth::kTokenHexLen + 1], uint32_t* info);

// /update icin 60 sn gecerli tek kullanimlik bilet.
void auth_issue_upload_ticket(char tokenHex[evse_auth::kTokenHexLen + 1]);
bool auth_check_upload_ticket(const String& ticketHex, bool consume);

// Varsayilan parola hala kullaniliyorsa true (panelde uyari bandi gosterilir).
bool auth_must_change_password();
uint32_t auth_session_ttl_sec();
