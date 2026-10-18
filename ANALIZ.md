# Firmware ve arayüz incelemesi

Bu not, bu daldaki firmware ile kartın web uygulamasını (PWA, QR, planlı şarj, oturum, Telegram, OTA) kod üzerinden okuyarak çıkarılmıştır. Cihaza yüklenmedi, şarj ve ağ üzerinde denenmedi. Bu turda ürün davranışı değiştirilmedi.

Kodda zaten duran korumalar: açılışta kontaktör LOW başlar; OTA yalnız `ota_0` / `ota_1` bölümüne yazar, factory bölümüne yazmaz; ardışık watchdog resetinden sonra factory bölümüne dönüş sayacı vardır; yönetim jetonu RAM’dedir, parola NVS’de tuzlu özettir; bulut varsayılan kapalıdır ve açılırsa kök sertifika kullanır. Aşağıdaki maddeler bu zeminin üstündeki boşluklardır.

Efor: **küçük** tek modül veya belge, **orta** birkaç modül ve kart üstü doğrulama, **büyük** donanım, anahtar altyapısı veya standart doğrulaması.

## Kritik

### 1. OTA’nın kimliği doğrulanmıyor ve yeni sürüm kendiliğinden kuruluyor

`src/OTA_Manager.cpp` içindeki `configureClient()` TLS sertifikasını kapalı kullanır (`setInsecure`). `version.json` imzasızdır. SHA-256 alanı varsa indirilen dosyayla karşılaştırılır; alan boşsa kontrol atlanır. Özet, aynı imzasız listenin içindedir; listeyi değiştiren kişi özeti de değiştirir. Uzaktaki sürüm numarası büyükse ve kontaktör kapalıysa `handleUpdateCheck()` kurulumu kullanıcı onayı beklemeden başlatır. Yönetim ekranındaki metin ayrıca onay ister der; kod otomatik kurar.

Neden: ev ile GitHub arasındaki yolda veya depo yazma yetkisinde duran biri kartın şarj yazılımını değiştirebilir. Şarj cihazında bu, kontaktör ve PWM davranışının değişmesi demektir.

Efor: **büyük**. İmzalı liste (ed25519 gibi), TLS doğrulaması ve “yeni sürüm bulundu, kur” onayı. Otomatik kurulum varsayılan kapalı kalmalı. Mevcut SHA-256 kontrolü imzanın yerine geçmez.

### 2. Telegram jetonu depoda ve TLS doğrulamasız; komutlar şarjı ve OTA’yı etkiler

`include/secrets.h` bot jetonunu ve sohbet kimliğini düz metin tutar ve depoya girmiştir. `src/telegram_notify.cpp` isteklerde `setInsecure()` kullanır. `guncelle` kontrol ve kurulumu tetikler, `geri al` ve `yeniden başlat` kartı yeniden başlatır. Sohbet kimliği, gövdede tam `"chat":{"id":` kalıbı bulunursa kontrol edilir. Kalıp yoksa komut yine de işlenir.

Neden: jetonu gören biri botu ele geçirir. TLS doğrulaması kapalı olduğu için ağdaki biri `getUpdates` yanıtını taklit edip `guncelle` veya yeniden başlatma yollayabilir. Bu, 1. maddedeki imzasız OTA ile birleşir.

Efor: **orta**. Jetonu depodan çıkarıp NVS’ye almak, TLS’i doğrulamak, sohbet kimliği ayrışmazsa komutu reddetmek, `guncelle` / `geri al` için paneldekiyle aynı onayı istemek. Eski jeton Telegram tarafında iptal edilmelidir; dosyadan silmek tek başına yetmez.

### 3. Yönetim ve Wi-Fi sırları düz metin HTTP üzerinde ve derlemenin içinde

Kart yalnızca HTTP sunar (`web_ui.cpp`, port 80). Oturum jetonu `sessionStorage` içindedir ve `Authorization` başlığıyla gider; çerez olmadığı için klasik siteler arası istek zayıftır, aynı ağda dinleyen ise başlığı görür. İlk parola `platformio.ini` içindeki `EVSE_ADMIN_PASSWORD` değeridir ve firmware ikilisinin içine girer. Ev ve ofis Wi-Fi parolaları da aynı dosyadadır. Özel Wi-Fi kaydı `GET /wifi_apply?...&wifiPass=` ile gider; parola adres çubuğuna ve günlük izine düşer. Giriş kilidi yalnız RAM’dedir: yeniden başlatma beş hatalık sayacı siler. Özet 1000 tur SHA-256’dır; NVS’yi okuyan biri için yavaşlatma kısadır.

Neden: aynı Wi-Fi’deki bir cihaz veya firmware dosyasını alan biri paneli, Wi-Fi parolalarını ve OTA yüklemesini kullanabilir. Varsayılan parola kaynakta yazılı olduğu sürece “ilk girişte değiştir” uyarısı bunu kapatmaz.

Efor: **orta** (parolaları ve Wi-Fi sırlarını NVS’ye taşımak, `wifi_apply` için POST, kilidi NVS’de tutmak). Kartın kendi sitesinde HTTPS **büyük**: telefona güvenilir sertifika dağıtmak veya yalnız yerel ağ tehdidini belgeleyip ağı bölmek gerekir. PWA’nın “Ana ekrana ekle” düğmesi de güvenli bağlam ister; `http://evse.local` birçok tarayıcıda bu düğmeyi hiç çıkarmaz.

### 4. Panelden STOP kontaktörü açıp hemen geri kapatabilir; 60 saniye sonra şarj yine başlar

Kontaktör kuralı bilinçli: kararlı state C veya D ise GPIO4 HIGH kalır, durdurma PWM ile yapılır (`src/io/relay.cpp`). Bulut durdurması bu yüzden `relay_force_off_now` çağırmaz. Paneldeki STOP (`handleChargeCmd`, mod 2) ise PWM’i kapatır ve aynı anda `relay_force_off_now()` çağırır. Otomatik mod açık kaldığı için sonraki ölçüm turu state hâlâ C/D ise çıkışı yeniden HIGH yapar. Ayrıca `g_manualStopAutoResumeAtMs` 60 saniye sonraya kurulur; süre dolunca mod AUTO olur ve araç B/C/D’deyse PWM geri gelir. Bulut STOP’u bu 60 saniyelik dönüşü kurmaz.

Neden: durdurma, kullanıcının beklediği kalıcı kesinti değildir. C/D sürerken kontaktör bir tur bırakılıp yeniden çekilir. 60 saniye sonra şarj, kimse START demeden devam edebilir.

Efor: **küçük** kod, **orta** doğrulama. Panel STOP’u bulut STOP’u ile aynı yola almak (kontaktöre dokunmamak veya C/D bitene kadar otomatiği kilitlemek) ve 60 saniyelik dönüşü ya kaldırmak ya da ekranda açıkça “60 saniye duraklat” diye göstermek. Kart üzerinde CP C iken osiloskop veya kontaktör sesi ile bakılmalı.

### 5. Toprak kaçağı, sıcaklık ve CP diyot denetimi yok

`include/app_pins.h` içinde RCD/GFCI, PE veya sıcaklık girişi yok. Aşırı akım yalnız alarm metnidir (`computeAlarm`); PWM’i veya kontaktörü kesmez. `pilot.cpp` state’i CP’nin yüksek seviyesinden seçer. Düşük yarı (`cpLowVolt`) ölçülür, state kararına girmez. `decideStateHysteresis` hiç `"F"` döndürmez. PP (GPIO16) yönetim JSON’unda ham ADC olarak durur; kablo akımını limite bağlanmaz. Limit, araca PWM duty ile söylenir.

Neden: diyotsuz veya yapışmış bir CP, araç yokken şarj gerilimine yol açabilir. Kablo ısınması, kaçak akım ve ortam sıcaklığı firmware tarafından kesilmez. Harici bir kaçak akım rölesi veya sigorta panoda olabilir; bu depoda onun izlendiğine dair kod yoktur.

Efor: **büyük**. Kaçak akım ve sıcaklık için donanım girişi, PP’den kablo sınırını okuyup duty’yi onunla kısıtlamak, diyot penceresi (state F) ve eşiklerin kartta ölçülmesi. Yazılım tek başına bu korumayı tamamlamaz.

## Önemli

### 6. kWh sayacı tahmin, geçmiş ise elektrik kesilince siliniyor

Enerji `src/main.cpp` içinde hesaplanır: hat gerilimi sabit 400 V kabul edilir, faz gerilimi 400/√3, güç faktörü 1. Gerilim veya gerçek güç ölçülmez. Örnekleme aralığı yaklaşık 200 ms. Sayaç ve son 20 seans RAM’dedir (`g_hist*`, `energyWh`). Yeniden başlatma, kesinti ve “geçmişi sil” hepsini sıfırlar. Yönetim tablosundaki başlangıç, takvim zamanı değil açılıştan beri geçen saniyedir (`fmtDate` 1e9 altını “s” diye yazar). Kullanıcı sayfası son 5 seansı süre ve kWh olarak gösterir; maliyet her zaman “—”.

Neden: ekrandaki kWh fatura veya karşılaştırma için güvenilmez. Kesinti, kullanıcının “dün ne kadar çekti” sorusunu da siler. Şarjın kendisi açılışta kontaktör LOW olduğu için kesintide enerji kesilir; araç hâlâ B/C/D ise AUTO mod şarjı yeniden başlatır. Bu, kesinti sonrası beklenmedik devamdır.

Efor: **orta**. Seansı NTP zamanı ve kWh ile NVS’ye veya halka tampona yazmak, açılışta yarım seansı kapatmak, kesinti sonrası AUTO devamını ekranda söylemek. Gerçek enerji için gerilim ölçümü **büyük** ve donanım ister. Sabit 230 V katsayısını belgelemek küçük bir ara adımdır.

### 7. Wi-Fi kopunca şarj sürer; yeniden bağlanma sert ve kurtarmasız

Şarj kararı Wi-Fi’ye bağlı değildir (buluttan “yalnızca uygulamadan başlat” kapalıyken). Kopunca PWM ve kontaktör aynı kalır. `web_tick` 10 saniyede bir `WiFi.begin` veya `wifiMulti.run(1000)` dener. mDNS kopunca durur, IP gelince `evse.local` yeniden kurulur. Özel profil yanlış kaydedilirse kart o ağa kilitlenir; bilinen listeye dönmek için panel gerekir, panele ise o ağdan girilir.

Neden: uygulama ve Telegram kopunca kaybolur, şarj devam eder. Kullanıcı bunu hata sanabilir. Yanlış Wi-Fi kaydı kartı uzaktan erişilemez bırakır. `wifiMulti.run(1000)` web görevini bir saniyeye kadar meşgul eder.

Efor: **orta**. Kopma sırasında OLED veya LED’de IP’yi göstermek, yanlış profil için bir düğme ile bilinen listeye dönüş, yeniden denemeyi web görevinden ayırmak. Şarjın kopmada sürmesi, bulut kapalıyken doğru davranıştır; ekranda “şarj sürüyor, telefon bağlantısı yok” denmesi küçük bir metin işidir.

### 8. Testler şarj, sayaç, OTA ve arayüzü kapsamıyor

`pio test -e native` yalnız `auth_core`, `cloud_core` ve `schedule_core` için 29 test çalıştırır. Pilot eşikleri, kontaktör C/D kuralı, panel STOP’un `relay_force_off_now` etkisi, enerji integrali, OTA sürüm karşılaştırması, Telegram sohbet ayrıştırması, `/status_public` ve QR için test yoktur. GPIO test ortamı LED pinlerini yakar, şarj döngüsü değildir.

Neden: 4. maddedeki kontaktör titremesi ve OTA’nın otomatik kurulumu testler yeşilken duruyor. Güvenlik düzeltmesi kart görmeden regresyon üretir.

Efor: **orta**. Saf fonksiyonlara ayrılabilenler (sürüm karşılaştırma, state F kararı, enerji adımı, STOP’un röle politikası) host testine alınabilir. PWM, ADC ve kontaktör için kart veya kayıtlı iz gerekir; o kısım **büyük**.

### 9. Hata çoğu yerde alarm veya günlük; şarjı kesen bir politika yok

State E PWM’i kapatır. State F üretilmediği için o dal pratikte boştur. Limit üstü akım, Wi-Fi yok ve manuel STOP alarm bandına yazılır. OLED hata LED’i E/F ve kısa STOP uyarısında yanar. Bulut ve OTA hataları geri çekilme sayacı tutar. Web JSON’u `snprintf` ile kesilirse istemci sessizce bozuk gövde alabilir. Kullanıcı sayfasında bağlantı kopunca “yeniden deneniyor” yazar; sebep (kart kapalı, yanlış ağ, mDNS) ayrılmaz.

Neden: araç duty’yi dinlemezse akım alarmı şarjı durdurmaz. Kullanıcı kırmızı bir şey görmeyebilir. Bozuk JSON, PWA’yı “bağlantı yok” sanmasına yol açar.

Efor: **orta**. Sürekli limit üstü akımda PWM kesmek (eşik ve süre kartta ölçülmeli), JSON taşmasını reddetmek, alarm metnini kullanıcı cümlesine çevirmek. Termik kesme 5. maddeyle birlikte **büyük**.

### 10. Yedek ve geri yükleme yok; OTA “sağlam” sayması zayıf

Kalibrasyon, limit, plan, parola ve Wi-Fi NVS’de ayrı durur. Bunları dışa aktaran veya geri yükleyen bir uç yoktur. Fabrika bölümüne dönüş NVS’yi silmez; yazılım eski, ayar yeni kalabilir. OTA sonrası “sağlam” kararı `web_ready_for_ota_validation()` ile verilir: Wi-Fi var ve en az bir HTTP yanıtı gitmiş olmak yeter. CP, kontaktör veya sayaç kontrol edilmez. 30 saniye içinde bu da olmazsa factory bölümüne dönülür.

Neden: kalibrasyon kartta emek ister ve kaybı şarj ölçümünü bozar. Kötü bir OTA, web hâlâ cevap veriyorsa şarj hatasıyla kalıcılaşır.

Efor: **orta**. Ayarların JSON dışa aktarımı (parola ve Wi-Fi parolası hariç) ve içe aktarma. OTA onayına “CP state A ve kontaktör LOW” şartı küçük bir ek, fakat kartta denenmeli.

### 11. Yönetici ile kullanıcı aynı kapının iki yüzü

Herkese açık sayfa (`/`, `/status_public`) salt okunur: state, akım, kWh, istasyon adı, `evse.local` ve IP. Durdurma, limit, Wi-Fi, OTA ve röle yalnız `admin` jetonu ister. İkinci rol, misafir süresi veya “şarjı durdur ama ayarı değiştirme” yoktur. Kullanıcı sayfasında START/STOP düğmesi de yoktur. Servis testlerindeki SET/RESET düğmeleri bu kartta 404 döner (`RELAY_USE_LATCH_PINS` 0).

Neden: telefonu eline alan herkes durumu görür; durdurmak için yönetici parolası gerekir. Evde bu parola paylaşılırsa OTA ve Wi-Fi de açılır. 404’lük röle düğmesi yanlış pinlere basıldığı izlenimi verir.

Efor: **orta** bir “oturum” rolü (durdur / izle, OTA ve Wi-Fi yok). Latch düğmelerini bu kartta gizlemek **küçük**.

### 12. Belgeler kullanımı anlatır, sınırları anlatmaz

Kök `README.md` uygulamayı açmayı ve iki güncelleme yolunu anlatır. `PLAN.md` birleştirme kararlarını tutar. `docs/` geliştirici haritasıdır. Hiçbiri şunları söylemez: STOP 60 saniye sürer ve kontaktör C/D iken açık kalır; kWh tahminidir; OTA imzasız ve yeni sürümde otomatiktir; `version.json` hâlâ eski 1.87 dosyasını gösterir; jeton ve Wi-Fi parolaları kaynakta düz metindir; toprak kaçağı kart yazılımında yoktur; `evse.local` ikinci kartta çakışır.

Neden: bu dosya olmazsa bir sonraki değişiklik “ekrandaki durdur kalıcı kesinti” veya “kWh faturalanır” diye okunur.

Efor: **küçük**. README’ye “bilerek böyle” ve “henüz yok” başlıkları. Sırların değerlerini belgeye kopyalamadan dosya adını söylemek yeter.

## Olsa iyi olur

### 13. Bildirim yalnız Telegram ve kopunca susuyor

Şarj başladı, bitti, hata veya kesinti için telefon bildirimi yok. PWA servis işçisi statik dosya tutar, anlık bildirim izin istemez. Telegram Wi-Fi ve `setInsecure` ile gider; kopuk ağda mesaj birikir veya düşer. Kullanıcı sayfası 3 saniyede bir `/status_public` okur, sekme kapalıyken susar.

Neden: kablo takılı unutulan şarj veya gece biten seans görünmez. Ev içi kullanımda şart değil, uzaktayken eksik kalır.

Efor: **orta** (bitiş ve hata için Telegram, Wi-Fi yokken kuyruk). Web Push bu HTTP kökeninde güvenilir çalışmaz; önce 3. madde.

### 14. Arayüz tek dil ve tek araç görseli

Metinler Türkçe ve sayfanın içine gömülü. Dil dosyası yok. Araç görseli Togg webp’si. Maliyet alanı bilinçli boş. QR ve “Uygulamayı aç” doğru yerel adrese gider. iPhone kurulumu Paylaş menüsüne bırakılmış; Android düğmesi güvenli bağlam olmadan gizli kalır (3. madde).

Neden: başka dil konuşan kullanıcı yönetim panelinde OTA ve STOP’u yanlış okuyabilir. Ürün tek ev içinse düşük öncelik.

Efor: **orta** metinleri bir tabloya almak. Çeviri metinleri ayrıca. Araç görselini genel bir silüet yapmak **küçük**.

### 15. Çoklu kart, dışa aktarma ve erişilebilirlik

İkinci kart `evse.local` adını paylaşamaz; IP ile açılır, QR ise hep `evse.local` kodlar. Geçmiş CSV veya yedek dosyası yok. Kullanıcı sayfasında süre ve kWh var, takvim ve dışa aktarma yok. Odak çerçevesi ve 44 px düğmeler duruyor; kare kodun metin alternatifi adres satırı.

Neden: iki kartlı evde QR yanlış karta gider. Geçmiş telefondan saklanamaz.

Efor: **küçük** QR’ye IP seçeneği veya kart adını host adına eklemek (o zaman tek karttaki `evse.local` sözü değişir; ürün kararı). CSV **küçük**.

## Önce hangisi

1. Panel STOP’un kontaktörü titretebilmesi ve 60 saniyede kalkması (4). Kartta ölçülür, kod tarafı küçük.
2. OTA otomatik kurulumu ve `setInsecure` (1), Telegram jetonu ve aynı TLS ayarı (2). İkisi birlikte uzaktan yazılım değişikliğidir.
3. Kaynakta duran parolalar ve HTTP üzerinde jeton (3), belgelerin bu sınırları yazması (12).
4. Diyot, kaçak ve sıcaklık (5) donanım kararı olmadan kapanmaz. O gelene kadar kWh’nin tahmin olduğu (6) ve test boşluğu (8) yazılır.

Bu sıra ürün kararıdır. 4. maddedeki C/D kontaktör kuralının kendisi önceki kararla duruyor; değişmesi istenen şey, STOP’un o kurala rağmen `relay_force_off_now` çağırıp 60 saniye sonra unutulmasıdır.
