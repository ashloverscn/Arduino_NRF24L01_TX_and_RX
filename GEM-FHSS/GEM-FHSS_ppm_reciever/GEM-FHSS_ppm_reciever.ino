#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte BIND_PIPE[6] = "BIND1";
const byte DATA_PIPE[6] = "DATA1";
bool isBound = false;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  
  // Phase 1: Listen for Bind
  radio.setChannel(100);
  radio.openReadingPipe(1, BIND_PIPE);
  radio.startListening();

  Serial.println(F("RX: Waiting for Bind Request..."));

  while (!isBound) {
    if (radio.available()) {
      char msg[32] = "";
      radio.read(&msg, sizeof(msg));
      if (strcmp(msg, "BIND_REQ") == 0) {
        isBound = true;
        Serial.println(F("RX: Bind Received! Switching to FHSS..."));
      }
    }
  }

  // Phase 2: FHSS
  radio.stopListening();
  radio.openReadingPipe(1, DATA_PIPE);
  radio.startListening();
}

void loop() {
  static uint8_t pnTable[] = {20, 40, 60, 80};
  static uint8_t i = 0;

  radio.setChannel(pnTable[i]);
  
  unsigned long start = millis();
  bool received = false;

  // Listen for 250ms (slightly longer than TX 200ms delay)
  while (millis() - start < 250) {
    if (radio.available()) {
      char text[32] = "";
      radio.read(&text, sizeof(text));
      Serial.print(F("Received: ")); Serial.print(text);
      Serial.print(F(" on Ch: ")); Serial.println(pnTable[i]);
      received = true;
      break;
    }
  }

  i = (i + 1) % 4;
}