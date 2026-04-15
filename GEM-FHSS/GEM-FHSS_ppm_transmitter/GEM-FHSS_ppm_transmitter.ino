#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PPM_IN_PIN 2
#define CHANNELS 6
#define HOP_COUNT 16

RF24 radio(9, 10);

struct Payload {
  uint16_t ch[CHANNELS];
  uint32_t txID;
  uint8_t hopIdx;
};

Payload data;
volatile uint16_t ppmIn[CHANNELS];
volatile unsigned long lastMicros = 0;
volatile byte pulseIdx = 0;

// Unique TX ID (Change this for different transmitters)
uint32_t transmitterID = 0xDEADBEEF; 
uint8_t hopTable[HOP_COUNT];
uint8_t currentHop = 0;

void readPPM() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;
  if (diff > 3000) pulseIdx = 0;
  else if (pulseIdx < CHANNELS) {
    ppmIn[pulseIdx] = diff;
    pulseIdx++;
  }
}

void generateHopTable(uint32_t seed) {
  randomSeed(seed);
  for (int i = 0; i < HOP_COUNT; i++) {
    hopTable[i] = random(2, 124); // Spread across full 2.4GHz band
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_IN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_IN_PIN), readPPM, RISING);

  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Max sensitivity
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);        // AFHDS style: don't wait for ACKs during flight
  
  generateHopTable(transmitterID);
  
  // Binding
  radio.setChannel(100);
  radio.openWritingPipe(0xE8E8F0F0E1LL);
  Serial.println(F("Binding..."));
  
  unsigned long bindTimer = millis();
  while (millis() - bindTimer < 5000) { // Send ID for 5 seconds
    if (radio.write(&transmitterID, sizeof(transmitterID))) break;
    delay(10);
  }

  radio.stopListening();
  Serial.println(F("Link Active"));
}

void loop() {
  unsigned long startHop = micros();
  
  radio.setChannel(hopTable[currentHop]);
  
  noInterrupts();
  for(int i=0; i<CHANNELS; i++) data.ch[i] = ppmIn[i];
  interrupts();
  
  data.txID = transmitterID;
  data.hopIdx = currentHop;

  radio.startWrite(&data, sizeof(data), true); // Multicast mode (no ack)

  currentHop = (currentHop + 1) % HOP_COUNT;
  
  // Strict 10ms timing (100Hz Refresh)
  while (micros() - startHop < 10000); 
}