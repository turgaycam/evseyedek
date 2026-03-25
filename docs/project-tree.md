# Proje Agaci

## Kok Duzeyi
- `platformio.ini`
  Ne is yapar: Kart, env, upload port, monitor port ve build ayarlari burada tutulur.
  Ne zaman bakilir: Yeni kart, yeni port, farkli build secenegi veya test env ihtiyacinda.

- `include/`
  Ne is yapar: Ortak baslik dosyalari ve tum modullerin paylastigi sabitler burada tutulur.
  Ne zaman bakilir: Pin, config veya log seviyesinde ortak degisiklik yapacaksan.

- `src/`
  Ne is yapar: Uygulamanin asil calisan kodu burada bulunur.
  Ne zaman bakilir: Davranis, ekran, web, sensor veya role mantigi degisecekse.

- `docs/`
  Ne is yapar: Proje rehberi ve mudahale haritalari burada tutulur.
  Ne zaman bakilir: Projeye geri dondugunde nereden baslayacagini hizli bulmak icin.

## Include Dosyalari
- `include/app_pins.h`
  Ne is yapar: Butun donanim pin haritasi burada tutulur.
  Ne zaman bakilir: OLED, CP, role, akim sensoru, LED veya latch pini degistiginde.

- `include/app_config.h`
  Ne is yapar: Moduller arasi paylasilan runtime ayarlarin deklarasyonlari burada bulunur.
  Ne zaman bakilir: Yeni bir ayari birden fazla modulle paylasacaksan.

- `include/app_log.h`
  Ne is yapar: `LOGE`, `LOGW`, `LOGI`, `LOGD` makrolari ve log seviyesi burada tanimlidir.
  Ne zaman bakilir: Daha fazla veya daha az log almak istediginde.

## Src Dosyalari
- `src/main.cpp`
  Ne is yapar: Tum modulleri bir araya getirir, ana loop akisina karar verir.
  Ne zaman bakilir: Enerji, seans, LED, state davranisi veya genel akista degisiklik yaparken.

- `src/pilot/pilot.h` ve `src/pilot/pilot.cpp`
  Ne is yapar: CP ADC olcumu, state karari ve PWM uygulamasi burada yapilir.
  Ne zaman bakilir: A/B/C/D/E/F state yorumlamasi veya duty mantigi degiseceginde.

- `src/io/current_sensor.h` ve `src/io/current_sensor.cpp`
  Ne is yapar: 3 faz akim olcumu, RMS hesaplama ve filtreleme burada yapilir.
  Ne zaman bakilir: Kalibrasyon, noise filtreleme veya olcum periyodu degiseceginde.

- `src/io/relay.h` ve `src/io/relay.cpp`
  Ne is yapar: Role cikisi ve latch set/reset darbeleri burada yonetilir.
  Ne zaman bakilir: Role acma/kapama gecikmeleri veya guvenlik mantigi degiseceginde.

- `src/ui/oled_ui.h` ve `src/ui/oled_ui.cpp`
  Ne is yapar: OLED ekranin layout, metin, panel ve blink mantigi burada bulunur.
  Ne zaman bakilir: Ekranin gorunusu, state yazilari veya hata gostergesi degiseceginde.

- `src/net/web_ui.h` ve `src/net/web_ui.cpp`
  Ne is yapar: Wi-Fi, mDNS, OTA, web paneli ve JSON endpointleri burada toplanir.
  Ne zaman bakilir: Web panelde veri, ayar veya yeni API gerekiyorsa.

- `src/gpio_test_main.cpp`
  Ne is yapar: Ana firmware yerine hizli LED / GPIO testi icin kullanilan mini programdir.
  Ne zaman bakilir: Donanimi parcali test etmek istediginde.
