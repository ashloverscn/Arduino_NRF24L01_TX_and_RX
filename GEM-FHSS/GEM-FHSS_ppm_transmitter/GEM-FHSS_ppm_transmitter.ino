#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- CONFIGURATION ---
#define DEBUG_MODE 1  // Set to 1 for Terminal output, 0 for flight
#define PPM_IN_PIN 2
#define CHANNELS 6

RF24 radio(9, 10);

// Global State
volatile uint16_t ppmValues[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile unsigned long lastMicros = 0;
volatile byte currentChannel = 0;

const byte BIND_PIPE[6] = "BND01";
const byte DATA_PIPE[6] = "DAT01";
uint8_t pnTable[] = {22, 44, 66, 88, 33, 55}; // PN Hopping Table
uint8_t pnIndex = 0;

// Interrupt: Measure PPM Input
void readPPM() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) { 
    currentChannel = 0; // Sync Gap
  } else if (currentChannel < CHANNELS) {
    ppmValues[currentChannel] = diff;
    currentChannel++;
  }
}

void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
    Serial.println(F("TX Init... Waiting for RX"));
  }

  pinMode(PPM_IN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_IN_PIN), readPPM, RISING);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(15, 15);
  
  // Binding Handshake
  radio.openWritingPipe(BIND_PIPE);
  radio.setChannel(100);
  while (!radio.write("BIND", 4)) { delay(200); }

  if (DEBUG_MODE) Serial.println(F("Bound! Starting FHSS PPM Transmission."));

  radio.openWritingPipe(DATA_PIPE);
  radio.setRetries(0, 0); // Strict timing
  radio.stopListening();
}

void loop() {
  radio.setChannel(pnTable[pnIndex]);

  uint16_t packet[CHANNELS];
  noInterrupts(); // Atomic copy
  for (byte i = 0; i < CHANNELS; i++) packet[i] = ppmValues[i];
  interrupts();

  bool ok = radio.write(&packet, sizeof(packet));

  if (DEBUG_MODE) {
    Serial.print(F("Ch:")); Serial.print(pnTable[pnIndex]);
    Serial.print(F(" [1]:")); Serial.print(packet[0]);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
  }

  pnIndex = (pnIndex + 1) % 6;
  delay(20); // 50Hz Link Rate
}