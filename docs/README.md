# EVSE Docs

Bu klasor, projeye tekrar baktiginda "hangi dosya ne is yapiyor, degisiklik icin nereye gidecegim?" sorusuna hizli cevap vermek icin tutulur.

## Nereden Baslamali
- Projenin genel yerlesimini gormek icin: `docs/project-tree.md`
- Hangi degisiklik hangi dosyadan yapilir ogrenmek icin: `docs/change-points.md`
- Kodun setup ve loop akisina bakmak icin: `docs/runtime-flow.md`

## Kisa Yonlendirme
- Yeni kart ve pin degisimi: `include/app_pins.h`
- OLED / ekran duzeni: `src/ui/oled_ui.cpp`
- Pilot, state ve PWM mantigi: `src/pilot/pilot.cpp`
- Akim olcumu ve kalibrasyon: `src/io/current_sensor.cpp`
- Role cikisi ve latch surusu: `src/io/relay.cpp`
- Ana is akisi ve runtime hesaplar: `src/main.cpp`
- Web panel ve API endpointleri: `src/net/web_ui.cpp`
- Kart, port ve derleme ayarlari: `platformio.ini`

## Not
Kod icindeki aciklayici yorumlar bu klasorle uyumlu olacak sekilde tutuldu. Buradaki rehber hizli yonlendirme icin, dosya icindeki yorumlar ise o modulu acinca ne oldugunu hemen anlaman icin var.
