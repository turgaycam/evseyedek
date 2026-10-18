# Şarj kartını nasıl kullanırım

Bu kart kendi başına çalışır. Telefonunuza ayrı bir mağaza uygulaması kurmazsınız. Uygulama, kartın içindeki web sayfasıdır. Telefonunuz kartla aynı ev Wi-Fi ağında olmalıdır.

## Uygulamayı nerede açarım

1. Telefonu, kartın bağlı olduğu Wi-Fi ağına alın.
2. Tarayıcıyı (Chrome veya Safari) açın.
3. Adres çubuğuna şunu yazın: `http://evse.local/`
4. Açılan sayfa uygulamadır. Şarj durumunu, gücü ve son seansları burada görürsünüz.

Sayfadaki **Uygulamayı aç** düğmesi ve kare kod aynı adrese gider: `http://evse.local/`. Sayfa bir de `http://192.168…/` biçiminde IP bağlantısı gösterir. `evse.local` açılmazsa o IP’yi kullanın.

`evse.local` bazı telefonlarda çözülmez. O zaman:

- Yönetici değilseniz, kartın Telegram botuna `durum` yazın. Gelen mesajdaki IP’yi tarayıcıya `http://` ile yazın.
- Ya da modemin bağlı cihazlar listesinden kartın IP’sine bakın.

Aynı Wi-Fi’de iki kart varsa ikisi de `evse.local` adını ister. İkinci kartı IP adresiyle açın.

### Telefonun ana ekranına ekleme

Sayfayı bir kez açtıktan sonra:

- **Android:** Sayfanın altındaki **Ana ekrana ekle** düğmesine basın. Düğme görünmezse tarayıcı menüsünden “Ana ekrana ekle” veya “Uygulamayı yükle”yi seçin.
- **iPhone:** Paylaş düğmesine basın, sonra **Ana Ekrana Ekle**. iPhone bu düğmeyi kendisi göstermez; Paylaş menüsü gerekir.

Bundan sonra simgeye basınca tarayıcı çubuğu olmadan açılır. Yine de telefonun kartla aynı Wi-Fi’de olması gerekir.

## Şarj nasıl başlar

Kabloyu araca takın. Kart buluta sormadan şarjı başlatır. Araç hazır olunca enerji akar. Durumu bu sayfadan izleyin.

Durdurmak için sayfanın altındaki **Yönetici girişi**ne girin (`http://evse.local/admin`). İlk kullanıcı adı `admin`, ilk parola derlemede kayıtlı olan paroladır (`platformio.ini` içinde `EVSE_ADMIN_PASSWORD`, şu an `rotosis123`). İlk girişten sonra panelden parolayı değiştirin (en az 10 karakter).

Planlı şarj da yönetim panelindedir. Varsayılan kapalıdır. Açarsanız seçtiğiniz saatlerde şarj bekler. Kart saati henüz bilinmiyorsa bu kural uygulanmaz.

Kontaktör, araç şarj konumundayken (C veya D) kapalı kalır. Durdurma, karta “şarjı kes” demek için PWM’i kapatır; araç o konumdan çıkınca kontaktör açılır.

## Telegram

Kart, daha önce olduğu gibi Telegram’dan da yanıt verir. Aynı bottaki birden fazla kartta komutun başına kart adını yazın (`hepsi` de olur). Kısa komutlar: `yardım`, `durum`, `bilgi`, `güncelle`, `geri al`, `yeniden başlat`, `log`, `isim`.

## Güncellemeyi nasıl yaparım

İki yol vardır. İkisi de kartın kendi yönetim panelinden başlar: `http://evse.local/admin` → yazılım güncelleme.

### 1. Dosyayı kendiniz yüklemek (internet gerekmez)

1. Yeni `firmware.bin` dosyasını telefona veya bilgisayara indirin.
2. Yönetim panelinde **.bin yükle** düğmesine basın.
3. Dosyayı seçin.

Kart dosyayı kendi `/update` adresinden alır, şarjı durdurur ve yeniden başlar. Ev interneti kapalı olsa da olur; telefonun karta erişmesi yeter.

### 2. GitHub Releases’ten (internet gerekir)

Kart, yaklaşık 10 dakikada bir şu listeye bakar:

`https://raw.githubusercontent.com/turgaycam/evseyedek/main/version.json`

Bu dosya sürüm numarasını ve firmware adresini söyler. Asıl `.bin` dosyası, `version.json` içindeki `url` alanındadır. Yeni bir sürüm etiketi (`v…` veya sayı) GitHub’a itilince Actions şunları yapar:

- Firmware’i derler.
- `firmware.bin` dosyasını o etiketin GitHub Release’ine koyar.
- `version.json` içindeki adresi `https://github.com/turgaycam/evseyedek/releases/download/<etiket>/firmware.bin` yapar.
- Aynı `version.json` dosyasını `main` dalına yazar.

Panelde **Kontrol et** yeni sürümü gösterir. **Güncellemeyi yükle** ayrıca onay ister. Uzaktaki sürüm karttakine eşitse kart kendiliğinden kurmaz.

Şu an depodaki `version.json` hâlâ `1.87` der ve eski yedek dalındaki dosyayı gösterir. Bu yüzden kart aynı sürümü yeniden kurmaz. Yeni bir Release yayınlanıp `main` üzerindeki `version.json` güncellenmeden **Güncellemeyi yükle**ye basmayın: eşit sürümde zorlanan kurulum eski dosyayı indirebilir.

## Bilinmesi gerekenler

- Bulut bağlantısı isteğe bağlıdır ve kapalı gelir. Adres boştur. Kart, siz panelden bir `https` adresi ve anahtar yazmadıkça hiçbir şarj sunucusuna bağlanmaz. “Şarjı yalnızca uygulamadan başlat” kutusu da o zamana kadar bir şey değiştirmez.
- İsterseniz kendi bulutunuz `server/` klasöründedir. Kurulum `server/KURULUM.md` içindedir. Kartın paneline yazılacak adres `https://turgay.rotosis.com` olur. Sunucu olmasa da kart şarj eder.
- Pin ve röle düzeni `include/app_pins.h` dosyasındadır. Başka bir kartın pinleri kullanılmaz.
- Geliştirici notları `docs/` ve `PLAN.md` içindedir.
