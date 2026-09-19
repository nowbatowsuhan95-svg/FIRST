// Arduino Uno + MFRC522 RFID reader.
// Reads a card's UID and forwards it to the ESP32 over SoftwareSerial.
// The ESP32 does all the WiFi / student lookup / logging work; this
// board only ever reports "a card with this UID was tapped".
//
// Wiring (MFRC522 -> Uno):
//   SDA/SS -> D10   SCK  -> D13   MOSI -> D11   MISO -> D12
//   RST    -> D9    3.3V -> 3.3V  GND  -> GND
//   (MFRC522 is a 3.3V part - do not power it from Uno's 5V pin)
//
// Wiring (Uno -> ESP32), one direction only:
//   Uno D3 (SoftwareSerial TX) -> voltage divider -> ESP32 GPIO16 (RX2)
//   Uno GND -> ESP32 GND
// The divider is required because the Uno's logic level is 5V and the
// ESP32's pins are not 5V tolerant. A simple 1k/2k divider on D3 works:
//   D3 --[1k]-- (node) --[2k]-- GND ; node -> ESP32 RX2

#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>

#define RFID_SS_PIN 10
#define RFID_RST_PIN 9
#define ESP32_SERIAL_RX_PIN 2 // unused (one-way link) but SoftwareSerial needs a pin
#define ESP32_SERIAL_TX_PIN 3 // -> voltage divider -> ESP32 RX2

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
SoftwareSerial esp32Serial(ESP32_SERIAL_RX_PIN, ESP32_SERIAL_TX_PIN);

String uidToString(const MFRC522::Uid &uid) {
  String result;
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) result += '0';
    result += String(uid.uidByte[i], HEX);
  }
  result.toUpperCase();
  return result;
}

void setup() {
  Serial.begin(9600);
  esp32Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
  Serial.println(F("RFID reader ready."));
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  String uid = uidToString(rfid.uid);
  Serial.print(F("Card tapped, UID: "));
  Serial.println(uid);

  esp32Serial.print(F("UID:"));
  esp32Serial.println(uid);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(1500); // debounce so one tap doesn't register as several reads
}
