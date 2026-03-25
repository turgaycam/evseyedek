# Mudahale Noktalari

Bu dosya, "su ozelligi degistirecegim, hangi dosyaya gitmeliyim?" sorusu icin hizli rehberdir.

## Donanim ve Derleme
- Yeni karta gectin, pinler degisti: `include/app_pins.h`
- Kart modeli, upload portu veya monitor portu degisecek: `platformio.ini`

## Sarj Mantigi
- CP state esikleri veya olcum ritmi degisecek: `src/main.cpp` ve `include/app_config.h`
- PWM / pilot state mantigi degisecek: `src/pilot/pilot.cpp`
- Role gecikmesi veya latch cikis mantigi degisecek: `src/io/relay.cpp`
- Akim sensor kalibrasyonu degisecek: `src/io/current_sensor.cpp`

## Arayuz
- OLED gorunumu degisecek: `src/ui/oled_ui.cpp`
- Web arayuzu veya yeni endpoint eklenecek: `src/net/web_ui.cpp`

## Test ve Servis
- Sadece LED / GPIO testi yapacaksan: `src/gpio_test_main.cpp`
- Log seviyesi veya log formati degisecek: `include/app_log.h`

## Pratik Kural
- Donanim degistiyse once `app_pins.h`
- Davranis degistiyse once `main.cpp`
- Modulun kendi ic mantigi degistiyse ilgili `*.cpp`
- Ekran degistiyse `oled_ui.cpp`
- Web degistiyse `web_ui.cpp`
