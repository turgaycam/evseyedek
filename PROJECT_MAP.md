# EVSE Project Map

Bu dosya, projeye geri dondugunde hizli baslangic noktasi olsun diye kokte tutulur.
Kod davranisini degistirmez; sadece nerede ne oldugunu hatirlatir.

## Ilk Bakilacak Yerler
- Kart, port, build ayarlari: `platformio.ini`
- Pin haritasi: `include/app_pins.h`
- Ortak runtime ayarlari: `include/app_config.h`
- Ana is akisi: `src/main.cpp`
- Pilot ve state mantigi: `src/pilot/pilot.cpp`
- Akim olcumu: `src/io/current_sensor.cpp`
- Role ve latch surusu: `src/io/relay.cpp`
- OLED ekrani: `src/ui/oled_ui.cpp`
- Web panel ve endpointler: `src/net/web_ui.cpp`
- Telegram: `src/telegram_notify.cpp`
- Bulut / mobil uygulama: `src/net/cloud_client.cpp` (mantik `cloud_core`)
- Planli sarj: `src/net/schedule.cpp` (mantik `schedule_core`)
- Yonetim oturumu: `src/net/auth.cpp` (mantik `auth_core`)
- Sadece GPIO testi: `src/gpio_test_main.cpp`
- Birlestirme karari: `PLAN.md`

## Hizli Mudahale Rehberi
- Yeni kart takildiysa once `include/app_pins.h`, sonra `platformio.ini`
- CP state esikleri degisecekse `src/main.cpp` ve `src/pilot/pilot.cpp`
- PWM veya state gecis mantigi degisecekse `src/pilot/pilot.cpp`
- Role gecikmesi degisecekse `src/io/relay.cpp`
- Akim kalibrasyonu degisecekse `src/io/current_sensor.cpp`
- Ekran gorunumu degisecekse `src/ui/oled_ui.cpp`
- Web panelde veri veya buton degisecekse `src/net/web_ui.cpp`

## Akis Ozeti
1. `setup()` icinde web, OLED, sensor, role ve pilot modulleri baslatilir.
2. `loop()` icinde web servisleri, sensor olcumu ve pilot state guncellenir.
3. Enerji, guc ve seans hesaplari `src/main.cpp` tarafinda uretilir.
4. OLED, LED, PWM ve role kararlari yine `src/main.cpp` icinden dagitilir.

## Detayli Rehber
Daha ayrintili dosya agaci ve mudahale notlari icin:
- `docs/README.md`
- `docs/project-tree.md`
- `docs/change-points.md`
- `docs/runtime-flow.md`
