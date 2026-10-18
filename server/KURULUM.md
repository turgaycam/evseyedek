# Evse bulut sunucusu — Plesk kurulumu

Bu paket, şarj kartının isteğe bağlı bulutudur. Adres olarak `https://turgay.rotosis.com` düşünülmüştür. Kart, bu sunucu olmadan da kendi Wi-Fi ağında şarj eder. Bulutu panelden açmadığınız sürece kart bu siteye bağlanmaz.

Sunucu Windows IIS 10 ve ASP.NET 4.7.2 (veya 4.8) içindir. PHP veya Node gerekmez. Veritabanı kurulmaz: kayıtlar `App_Data` içindeki dosyalardadır. Ek DLL kopyalamanız gerekmez; JSON ve parola özeti .NET Framework’ün kendi sınıflarıdır.

## Paket

FTP’ye yüklenecek dosya: `server/evse-bulut-iis.zip`. Yanındaki `evse-bulut-iis.sha256` dosyası bütünlük kontrolüdür.

Zip’in içinde site kökü vardır (`web.config`, `default.aspx`, `App_Code`, `App_Data`) ve bu kılavuz. `web.config` doğrudan `httpdocs` içinde durmalıdır. Kılavuzu siteye koymasanız da olur. Koyarsanız sayfadan okunur; içinde parola yoktur.

## 1. Plesk’te site

1. `turgay.rotosis.com` alan adı Windows barındırmada olsun. Uygulama havuzu **.NET CLR v4**, bütünleşik (integrated) kip, **tek** çalışan işlemi.
2. ASP.NET sürümü **4.7.2 veya 4.8**. 4.6 bu kodu derlemez.
3. Sertifika **Let’s Encrypt** olsun. Kartın içindeki güven deposu ISRG kökleridir. Başka bir sertifika markası (özel kök, bazı ticari zincirler) kartta TLS hatası verir. Plesk’te SSL/TLS ayarında siteye Let’s Encrypt bağlayın ve HTTP’yi HTTPS’e yönlendirin.
4. Kart yalnızca `https://` kabul eder. Adresin sonuna yol eklemeyin.

## 2. Dosyaları koyma

1. Zip’i bilgisayarda açın.
2. Plesk Dosya Yöneticisi veya FTP ile `httpdocs` içine yükleyin. `web.config` doğrudan `httpdocs/web.config` olmalı.
3. `App_Data` klasörüne uygulama havuzunun **değiştirme** iznini verin. Plesk’te klasöre sağ tıklayıp izinlerden, site kullanıcısının yazabildiğini işaretleyin. Bu olmazsa kurulum “yazılamadı” der.
4. `https://turgay.rotosis.com/App_Data/oku.txt` açılmamalıdır (IIS `App_Data` adını gizler). Açılıyorsa yüklemeyi durdurun.

## 3. Yönetici parolası

1. `https://turgay.rotosis.com/kurulum.aspx` adresini siteyi duyurmadan açın.
2. Kullanıcı adı ve en az 10 karakterlik parola belirleyin. Parola dosyada düz metin durmaz; PBKDF2-SHA256 özeti `App_Data/yonetici.json` içine yazılır.
3. Ardından `yonetim/giris.aspx` ile girin. Kurulum sayfası ikinci kez parola koymaz.
4. Parolayı unutursanız FTP’den yalnız `App_Data/yonetici.json` dosyasını silin ve kurulum sayfasını yeniden açın. İstasyon kayıtları `App_Data/cihazlar` altındadır, o klasörü silmeyin.

Oturum çerezi yalnız HTTPS’te gönderilir. Siteyi HTTP ile denerseniz giriş tutmaz.

## 4. İstasyon kaydı

1. Yönetimde **Yeni istasyon**. Adı yazın. Kodu boş bırakırsanız kartın kabul ettiği biçimde üretilir (3–16 karakter, büyük harf, rakam, `_` veya `-`).
2. Açılan sayfadaki **gizli anahtarı hemen kopyalayın**. 43 karakterdir, base64url. Sayfa yenilenince bir daha gösterilmez. Dosyada yalnız özeti durur.
3. Herkese açık durum bağlantısı aynı sayfada yazar: `https://turgay.rotosis.com/durum.aspx?kod=KOD`. Bu sayfa şarj başlatmaz.

## 5. Karta yazılacaklar

Kartın yönetim paneli → **Bulut bağlantısı**:

| Alan | Değer |
| --- | --- |
| Bulut bağlantısı açık | işaretli |
| Sunucu adresi | `https://turgay.rotosis.com` |
| İstasyon kodu | yönetimde görünen kod |
| Gizli anahtar | bir kez gösterilen 43 karakter |

Kaydettikten sonra kartta durum bir süre **saat bekleniyor** kalabilir. Kart, saati NTP’den almadan TLS açmaz. Sonra yaklaşık her 2,5 saniyede bir `POST /api/device/poll` yapar. Yönetimde rozet **çevrimiçi** olursa yol tamamdır.

**Şarjı yalnızca uygulamadan başlat** kutusunu boş bırakın. O zaman araç takılınca kart yine kendi kendine şarj eder; bu sitedeki Başlat/Durdur ek bir kumandadır. Kutuyu işaretlerseniz şarjın başlaması için bu siteden Başlat gerekir. Şarj başladıktan sonra sunucu veya Wi-Fi kopsa da kart şarjı kesmez.

Kartın kare kodu bu siteye gitmez. Kare kod ev ağındaki `http://evse.local/` adresidir. Bu sitedeki durum sayfası, o ekranın uzaktan karşılığıdır.

## 6. Yoklama neden kırmızı kalır

Kartın yönetim ekranındaki bulut satırı:

| Yazı | Anlamı |
| --- | --- |
| kapalı | Panelde bulut kapalı. Sunucuyla ilgisi yok. |
| ayar yok | Adres, kod veya anahtar eksik. |
| saat bekleniyor | Kart henüz saat almadı. |
| bağlı | Yoklama kabul edildi. |
| anahtar reddedildi | HTTP 401. Kod veya anahtar bu sitedeki kayıtla uyuşmuyor. |
| hata | Adres, sertifika, ağ veya yanıt biçimi. |

Sertifika Let’s Encrypt değilse kart `hata` der, parola doğru olsa bile.

## 7. Protokol uyumu

Kart kodu `src/net/cloud_core.cpp` içindedir. Bu sitenin uyduğu kurallar:

- Adres `https://` ile başlar, en çok 95 karakterdir. `https://turgay.rotosis.com` bunun içindedir. Sondaki `/` kart tarafından silinir. Kart yolu kendisi ekler: `/api/device/poll`.
- İstek JSON alanları: `code`, `secret`, `state`, `ia`, `ib`, `ic`, `pW`, `eKWh`, `tSec`, `fw`, `paused`, `pauseReason`, `resumeInSec`, `sched`, varsa `ackId` ve `ackOk`.
- Yanıt en fazla 1024 bayttır. Komut yokken tam olarak `{"ok":true,"t":<unix>,"cmd":null}` biçimindedir. `ok` true/false sözcüğüdür, tırnaksızdır.
- Komut varken `cmd` şöyledir: `{"id":41,"type":"start"}`. `id` sıfırdan büyük tam sayıdır. `type` bu panelden yalnız `start` veya `stop` olur.
- Kart onaylayana kadar aynı `id` yinelenir. Onay `ackId` + `ackOk` ile gelir. `ackOk` false ise kart komutu reddetmiştir (örneğin araç bağlı değilken Başlat); aynı komut tekrar gönderilmez.
- `set_limit` kart tarafından ayrıştırılır ve **uygulanmaz** (onay false döner). Bu panel onu kuyruğa koymaz.
- Kod 3–16 karakter, yalnız `A-Z`, `0-9`, `_`, `-`. Anahtar tam 43 karakter, `A-Za-z0-9_-`.

Kart firmware’indeki çözümleyici, bu sitenin ürettiği üç örnek yanıtı kabul eder. Bu, depodaki `pio test -e native` ile bakılır. Kartın kendisi bu sunucuya bağlanarak denenmedi.

## 8. Yedek

`App_Data` klasörünü kopyalamak yedektir. Gizli anahtarın düz metni bu klasörde yoktur. Yedekten geri dönünce karttaki anahtar duruyorsa giriş sürer. Anahtarın düz metnini kaybettiyseniz istasyonu silip yenisini oluşturun ve kartın anahtar alanına yenisini yazın.

## Bu ortamda denenemeyenler

- Plesk, IIS 10 ve Let’s Encrypt ile gerçek yayın.
- ESP32’nin bu alan adına TLS el sıkışması ve 2,5 saniyelik yoklama.
- Başlat/Durdur’un araçta kontaktörü nasıl etkilediği. Kartta durdurma PWM ile olur; kontaktör kuralı değişmedi.
- FTP yüklemesi ve `App_Data` izin ekranı.

Yerelde çalışan doğrulama: protokol sınıflarının birim testi (`server/test/Test.cs`) ve kart firmware’inin bu yanıt metinlerini kabul ettiği yerel test.
