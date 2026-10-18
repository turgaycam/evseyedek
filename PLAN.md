# Birleşik EVSE firmware planı

Bu dosya, kod değişmeden önce iki reponun okunmasıyla çıkan kararı ve ardından uygulanacak işi kaydeder.
Kaynaklar: bu repo (`turgaycam/evseyedek`, sürüm 1.87) ve `RotosisRobotics/RotosisEVSE` (sürüm 1.1.66, `main`).

## Bu repoda ne var (yeni kart, doğru donanım)

ESP32-S3, PlatformIO, Arduino. Şarj güvenliği şu zincirle yürür: CP ölçümü (`src/pilot/pilot.cpp`) → PWM kararı (`src/main.cpp`) → kontaktör (`src/io/relay.cpp`).

Pin ve röle tanımı tek yerde, `include/app_pins.h`:

| İşlev | GPIO | Not |
| --- | --- | --- |
| CP PWM | 2 | 1 kHz, 10 bit |
| CP ADC | 1 | |
| PP ADC | 16 | Yeni kartta var |
| Kontaktör | 4 | Aktif-high: HIGH = AC, LOW = kapat |
| Akım L1 / L2 / L3 | 5 / 6 / 7 | |
| OLED SDA / SCL | 18 / 17 | |
| State / Wi-Fi / Hata LED | 13 / 12 / 14 | |
| RGB | 15 | Uygulama mantığında kullanılmıyor |

`RELAY_USE_OUTPUT_PIN 1`, `RELAY_USE_LATCH_PINS 0`. Eski SET/RESET pin adları (GPIO7 ve GPIO16) bilinçli olarak kapalıdır; GPIO7 L3 akım, GPIO16 PP'dir. Latch açılırsa bu iki hat bozulur.

Kontaktör kuralı (bu kartta doğru olan): kararlı state C veya D ise GPIO4 hemen HIGH, diğer state'lerde hemen LOW. Gecikme yok, latch darbesi yok, PWM şartı yok. `relay_update_auto` PWM argümanını bilerek yok sayar.

Bunlara ek olarak bu repoda olup Rotosis'te olmayanlar:

- Telegram botu (`src/telegram_notify.cpp`, `include/secrets.h`). Komutlar ana döngüyü bloklamaz. `yardım`, `durum`, `bilgi`, `güncelle`, `geri al`, `yeniden başlat`, `log`, `isim`. Birden fazla istasyon aynı bota bağlıysa kart adı veya `hepsi` gerekir.
- Kart kimliği: MAC'ten isim (`Ters Lale`, `Toros Sediri`, diğerlerine endemik bitki adı).
- CP bölücü otomatik 12 V ayarı (yalnız state A, PWM kapalı) ve kullanıcının state eşiklerinin NVS'de kalıcı tutulması.
- Yönetim uçlarında `/logs`, PP ölçümü, HTTP Basic Auth.
- OTA manifesti bu reponun `version.json` adresine bakar.

GPIO test dosyasındaki dizi `{8, 17, 18}` eski kart pinleridir; ekrana yazılan metin ise yeni kartı söyler. Düzeltilecek: test, `app_pins.h` makrolarını kullanacak.

## RotosisEVSE'de ne var (eski kart)

Aynı ürünün eski PCB firmware'i. Pin haritası bu kart için yanlıştır ve alınmayacak:

| İşlev | Rotosis GPIO | Bu kart |
| --- | --- | --- |
| OLED SDA / SCL | 14 / 13 | 18 / 17 |
| Kontaktör | 3 | 4 |
| Akım L1 / L2 / L3 | 4 / 5 / 6 | 5 / 6 / 7 |
| State / Wi-Fi / Hata LED | 8 / 17 / 18 | 13 / 12 / 14 |
| Latch SET / RESET | 15 / 7, 100 ms darbe | Kapalı |
| PP | Yok | GPIO16 |

Eski röle kuralı da farklıdır: C/D ve PWM açık ve duty > 0 iken, açmada 1000 ms / kapamada 500 ms gecikmeyle, ayrıca latch darbesiyle. Bu kural yeni karta taşınmayacak.

Rotosis'te Telegram yoktur. Wi-Fi parolaları `secrets.ini` ile derlemeye girer (git'te yok). Bu repodaki mevcut `platformio.ini` bayrakları duracak; aksi halde bu ortamda derleme kırılır.

## Rotosis'ten alınacak özellikler

Hepsi varsayılan kapalı veya anahtarsız pasiftir. Anahtar ve "uygulamadan başlat" kapalıyken şarj kararı bugünkü gibi kalır.

1. **İsteğe bağlı bulut** (`cloud_core` + `cloud_client`). Varsayılan kapalı, adres boş. Derlemede hiçbir sunucu adresi yoktur; kart dışarıya bağlanmaz. Kullanıcı panelden bir `https` adresi, istasyon kodu ve 43 karakterlik anahtar kaydedip açarsa, yaklaşık 2,5 sn'de bir `{adres}/api/device/poll` yapılır. Anahtar NVS'dedir, hiçbir JSON yanıtında geri dönmez. Komutlar: `start`, `stop`. `set_limit` ayrıştırılır ama uygulanmaz (protokolde amper alanı yok). `requireCloudStart` açıksa şarjın başlaması için bulut `start` gerekir; başladıktan sonra bağlantı kopsa da şarj kesilmez. İzin RAM'dedir, reset'te düşer.
2. **Kartüstü uygulama ve QR.** Kullanıcı sayfası (`/`) kartın kendi PWA'sıdır. mDNS adı tam olarak `evse` olduğu için adres `http://evse.local/` olur. `/qr.svg` bu adresi gerçek bir kare kod olarak üretir. "Uygulamayı aç" aynı adrese gider. IP ikinci bir bağlantı olarak gösterilir. Android'de "Ana ekrana ekle", iPhone'da Paylaş → Ana Ekrana Ekle. Rotosis bulutuna giden bir uygulama adresi yoktur.
3. **Herkese açık durum** `GET /status_public`. Kullanıcı ekranı bunu okur. Kalibrasyon, IP, MAC, OTA yoktur.
4. **Planlı şarj** (`schedule_core`). En çok 3 yasaklı saat aralığı, gece yarısını aşan aralık, gün bitmaskesi, UTC+3. Varsayılan kapalı (örnek aralık 17:00–22:00). Saat NTP'den gelmezse kısıtlama uygulanmaz.
5. **Yönetim oturumu** (`auth_core`). Basic Auth yerine `POST /api/login` → Bearer jeton (15 dk, en çok 4 oturum). Parola NVS'de tuzlu SHA-256. 5 hatada kilit. `POST /api/password`. OTA için 60 sn'lik tek kullanımlık bilet. İlk parola bugünkü `EVSE_ADMIN_PASSWORD` değeridir (10 karakter, minimumu karşılar).
6. **Yönetim API'leri:** `/api/cloud_cfg`, `/api/schedule`, `/api/logout`, `/api/upload_ticket`.
7. **Kullanıcı arayüzü:** RTevCharge herkese açık sayfa, Togg araç görseli, seans özeti, alarm, planlı şarj satırı. Yönetim paneli bulut ve zamanlayıcı kartları.
8. **Seans bekletme:** araç state B'ye dönerse (kablo takılı, PWM açık, hata yok) seans kapanmaz; fiş çıkınca veya STOP/hata ile biter. Röle kuralını değiştirmez.
9. **Host birim testleri** (`pio test -e native`): auth, bulut kapısı, zamanlayıcı.

## Nasıl birleştirilecek

- Pin, LED, OLED, PP, akım ve kontaktör: `include/app_pins.h` aynen kalır. Röle modülü C/D → anında HIGH kuralını korur. Latch pinleri çıkış yapılmaz.
- Pilot, akım sensörü, OLED, Telegram, kart kimliği ve OTA adresi bu repodan kalır.
- Bulut ve zamanlayıcı yalnızca PWM kapısını etkiler (Rotosis `main.cpp` ile aynı yer). İzin yoksa PWM kapanır. Araç C/D'den çıkınca mevcut röle kuralı GPIO4'ü LOW yapar.
- Rotosis `cloud_client` içindeki `relay_force_off_now` (3 sn sonra) bu karta taşınmayacak. Eski kartta röle PWM'e bağlı olduğu için orada kalıcı açılma olur. Bu kartta aynı çağrı, state hâlâ C/D ise kontaktörü bir tur kapatıp ertesi tur yeniden çeker. Titreme yaratmamak için yeni kartta durdurma PWM üzerinden yapılır.
- Web arayüzü Rotosis sürümünden alınır, üzerine şu yerel parçalar geri konur: PP ölçümü, `/logs`, state eşiği ve bölücü kaydı (`pilot_set_divider` / `pilot_save_state_thresholds`), latch uçlarının `RELAY_USE_LATCH_PINS` arkasında kalması. Eski özel CSS kaydı silinmez (Rotosis 1.1.64'te güvenlik için siliniyor); düzenleyici de geri gelmez.
- Yönetim uçları Bearer ister. Kullanıcı sayfası ve `/status_public` açık kalır.

## Bilinçli olarak alınmayanlar

- Rotosis pinleri, latch darbesi, röle gecikmesi, "röle yalnızca PWM açıkken" şartı.
- `secrets.ini` ayrıştırması (bu repodaki derleme bayrakları durur).
- `set_limit` komutunun akıma uygulanması (protokol alanı yanıtta yok; Rotosis de reddediyor).
- Kullanıcı CSS editörü.

## Karar verilen varsayımlar

1. Kontaktör C/D kuralında kalır. Uygulama, plan veya STOP yalnızca PWM'i kapatır. CP hâlâ C veya D iken kontaktörü zorla açmak bu kartın kuralını bozacağı için yapılmaz.
2. Uygulama adresi `http://evse.local/` dir. QR bunu kodlar. Sayfa IP'yi de gösterir. mDNS adı tam `evse` olduğu için aynı Wi-Fi'de ikinci kart `evse.local` ile çakışır; o kart IP ile açılır.
3. Bulut varsayılan kapalıdır, adres boştur, derlemede gömülü sunucu yoktur. İleride kullanıcı panelden kendi `https` adresini ve anahtarını yazarsa istemci o zaman çalışır.
4. Özel CSS düzenleyici geri gelmez. Eski `user_css` kaydı silinmez.
5. `set_limit` uygulanmaz. Akım limiti yönetim panelinden 6–32 A olarak kalır.
6. İlk yönetim kullanıcısı `admin`, ilk parola derlemedeki `EVSE_ADMIN_PASSWORD` değeridir. Panel değiştirmeyi ister.
7. Son seanslar kartın içinde durur. Kullanıcı sayfası son 5 seansı gösterir. Bulut hesabı yoktur.
8. "Şarjı yalnızca uygulamadan başlat" kutusu, adres ve anahtar kaydedilmeden hiçbir şeyi değiştirmez. Normal yol: araç takılınca otomatik şarj, yerel START/STOP.
9. Otomatik güncelleme `main` dalındaki `version.json` dosyasını okur. O dosyanın `url` alanı firmware'dir; etiketli GitHub Release bu dosyayı ve `firmware.bin` dosyasını yayınlar. Eşit sürüm kendiliğinden kurulmaz. Elle `.bin` yükleme (`/update`) internet istemez.
10. Pin ve röle kaynağı yalnızca `include/app_pins.h` olmaya devam eder.

## Uygulama durumu

Birleştirme ve bağımsızlık kararı uygulandı. Pin haritası ve C/D kontaktör kuralı duruyor. Telegram duruyor. QR, kullanıcı ekranı, planlı şarj, oturum, `/status_public` ve Telegram kartın içinde, yerel ağda çalışır. Bulut yoklaması varsayılan kapalıdır. Kullanıcı kılavuzu kök `README.md` dosyasındadır.
