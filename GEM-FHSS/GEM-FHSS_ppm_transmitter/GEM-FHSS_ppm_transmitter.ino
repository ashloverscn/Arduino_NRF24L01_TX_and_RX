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
  radio.setRetries(15, 15); // Maximum retries for binding
  
  // Phase 1: Binding
  radio.openWritingPipe(BIND_PIPE);
  radio.setChannel(100); // Use a quiet channel
  
  Serial.println(F("TX: Attempting to Bind..."));

  while (!isBound) {
    const char msg[] = "BIND_REQ";
    if (radio.write(&msg, sizeof(msg))) {
      isBound = true;
      Serial.println(F("TX: RX found! Starting FHSS in 1 second..."));
    }
    delay(250); // Don't flood the bus too fast
  }

  delay(1000); // Let RX settle
  radio.openWritingPipe(DATA_PIPE);
  radio.setRetries(0, 0); // Disable for FHSS
  radio.stopListening();
}

void loop() {
  // FHSS Hopping Table
  static uint8_t pnTable[] = {20, 40, 60, 80};
  static uint8_t i = 0;

  radio.setChannel(pnTable[i]);
  const char text[] = "Hello World";
  radio.write(&text, sizeof(text));
  
  Serial.print(F("Sent Hello on Ch: ")); Serial.println(pnTable[i]);
  
  i = (i + 1) % 4;
  delay(200); // Slower 5Hz hop for stability
}