# Talyp dowamat ulgamy (RFID + Arduino Uno + ESP32)

Talyplaryň uniwersitede girende RFID kartyny degirip geçmegini we
şoňa görä näçe gün sapaga gireni / näçe gün galanyny hasabatda görkezýän
ulgam.

## Arhitektura

```
[RFID kart] --degirilende--> [MFRC522 modul] --SPI--> [Arduino Uno]
                                                            |
                                                   SoftwareSerial (D3)
                                                   5V -> 3.3V bölüji arkaly
                                                            v
                                                       [ESP32 RX2]
                                                            |
                                          WiFi web-server + SPIFFS
                                          (students.json, attendance.csv)
```

- **Arduino Uno + MFRC522**: diňe kartyň UID-ini okaýar we ESP32-a iberýär.
  Wagt, talyp sanawy ýa-da hasabat bilen işi ýok — arzan, ygtybarly "okaýjy" bölegi.
- **ESP32**: WiFi-e birigýär, NTP arkaly dogry wagty alýar, gelen UID-i
  `students.json`-daky sanaw bilen deňeşdirýär, `attendance.csv`-a ýazýar
  we web-sahypa arkaly hasabat berýär.

## Simler (wiring)

**MFRC522 -> Arduino Uno:**

| MFRC522 | Uno  |
|---------|------|
| SDA/SS  | D10  |
| SCK     | D13  |
| MOSI    | D11  |
| MISO    | D12  |
| RST     | D9   |
| 3.3V    | 3.3V |
| GND     | GND  |

MFRC522 3.3V-lyk modul — Uno-nyň 5V pinine birikdirmäň.

**Arduino Uno -> ESP32** (diňe bir tarapa maglumat, Uno -> ESP32):

Uno-nyň D3 pini 5V logiki, ESP32-nyň pinleri bolsa 3.3V-e çydamly
(5V gelse zaýalanyp biler), şonuň üçin naprýaženiýe bölüji gerek:

```
Uno D3 --[1k rezistor]--+--[2k rezistor]-- GND
                          |
                          +--> ESP32 GPIO16 (RX2)
Uno GND --------------------> ESP32 GND
```

## Gerekli kitaphanalar (Arduino IDE Library Manager arkaly)

- `MFRC522` (miguelbalboa) — Arduino Uno taslamasy üçin
- `ArduinoJson`, **6.x wersiýasy** (bblanchon) — ESP32 taslamasy üçin
- ESP32 board paketi (`esp32` by Espressif Systems)

## Gurnama tertibi

1. `firmware/arduino_uno_rfid_reader/arduino_uno_rfid_reader.ino` faýlyny
   Arduino Uno-a ýükläň.
2. `firmware/esp32_attendance_hub/esp32_attendance_hub.ino` faýlynyň
   başyndaky `WIFI_SSID` we `WIFI_PASSWORD` üýtgeýänlerini öz WiFi
   maglumatlaryňyza görä üýtgediň.
3. `firmware/esp32_attendance_hub/data/students.json` faýlyny öz
   talyplaryňyzyň hakyky RFID kart UID-leri bilen dolduryň (UID-i ilkinji
   gezek näbelli kart hökmünde skanirläp, dashboard-daky "Tanalmadyk
   kartlar" linkinden hem goşup bolýar).
4. Arduino IDE-de "ESP32 Sketch Data Upload" guralyny ulanyp, `data/`
   bukjasyny ESP32-nyň SPIFFS ýadyna ýükläň (bu gural Arduino IDE-e
   aýratyn goşulmaly bolup biler — "arduino-esp32fs-plugin" gözläň).
5. ESP32 sketchini ESP32 platasyna ýükläň. Serial Monitor-da (115200 baud)
   ESP32-nyň WiFi IP salgysyny görersiňiz.

## Ulanyş

Brauzerde ESP32-nyň IP salgysyna giriň:

- `/` — umumy dashboard
- `/today` — şu gün kim geldi / kim gelmedi
- `/report` — her talyp üçin jemi gatnan/gatnamadyk gün sany we göterimi
- `/students` — bellenen talyplaryň sanawy
- `/register` — täze kart/talyp goşmak üçin forma
- `/raw` — `attendance.csv` faýlyny göni göçürip almak

**Hasaplama:** "jemi sapak güni" diýlip, ulgamda azyndan bir talybyň
skanirlenen ähli senesiniň sany alynýar. Her talyp üçin gatnamadyk gün
sany = jemi sapak güni - şol talybyň gatnan güni.

## Bilinmeli çäklendirmeler

- Bu kod hakyky enjamda synag edilmedi (bu gurşawda ESP32/Arduino
  enjamy ýok) — kompilirlemezden ozal simleri we kitaphana wersiýalaryny
  gaýtadan barlaň.
- `attendance.csv` we `students.json` SPIFFS-de saklanýar; ESP32-nyň
  flash ýady arkaly (birnäçe ýyllyk gündelik ulanyşda-da) ýeterlik bolar,
  ýöne wagtal-wagtal `/raw` arkaly ätiýaçlyk nusgasyny alyp durmak maslahat
  berilýär.
