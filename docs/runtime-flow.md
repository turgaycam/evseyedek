# Akis Rehberi

Bu dosya, sistemin acilis sirasini ve calisma dongusunu hizli okumak icin tutulur.

## Baslatma Sirasi
1. `setup()` icinde seri haberlesme acilir.
2. Web, Wi-Fi, mDNS ve OTA servisleri baslatilir.
3. OLED baslatilir.
4. Akim sensoru baslatilir.
5. Role modulu hazirlanir.
6. Pilot PWM ve ADC modulu baslatilir.
7. Ilk PWM durumu uygulanir.

## Runtime Sirasi
1. `web_loop()` cagrilir.
2. `current_sensor_loop()` ile olcumler tazelenir.
3. `pilot_update()` ile CP state okunur.
4. Akim, guc, enerji ve seans verileri hesaplanir.
5. OLED ve LED gosterimleri guncellenir.
6. PWM hedefi belirlenir.
7. Role ve latch cikislari guncellenir.

## Guvenli Degisiklik Rutini
1. Degisiklik alanini `docs/change-points.md` icinden bul.
2. Ilgili `*.h` dosyasina bakip modulun dis arayuzunu anla.
3. Sonra `*.cpp` dosyasinda akisi incele.
4. Gerekirse `src/main.cpp` icinde modulun ne zaman cagrildigini kontrol et.
5. Degisiklikten sonra derleme al.

## Hangi Dosyayi Once Acmali
- Ekranla ilgili bir sey degisecekse: `src/ui/oled_ui.cpp`
- State davranisi degisecekse: `src/pilot/pilot.cpp`, sonra `src/main.cpp`
- Role davranisi degisecekse: `src/io/relay.cpp`
- Sensor sonucu yanlissa: `src/io/current_sensor.cpp`
- Web panelde alan eksikse: `src/net/web_ui.cpp`
