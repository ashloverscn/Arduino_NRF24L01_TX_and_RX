#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PPM_IN_PIN 2
#define CHANNELS 6
#define HOP_COUNT 16
#define DEBUG_PPM 1

RF24 radio(9, 10);

struct Payload {
  uint16_t ch[CHANNELS];
  uint32_t txID;
  uint8_t hopIdx;
  uint8_t crc;
};

Payload data;

volatile uint16_t ppmIn[CHANNELS] = {1500,1500,1500,1500,1500,1500};
volatile unsigned long lastMicros = 0;
volatile byte pulseIdx = 0;

uint32_t transmitterID = 0xDEADBEEF;
uint8_t hopTable[HOP_COUNT];
uint8_t currentHop = 0;

unsigned long lastSendTime = 0;
unsigned long lastDebugTime = 0;

// --- CRC ---
uint8_t calcCRC(Payload &p) {
  uint8_t sum = 0;
  for (int i = 0; i < CHANNELS; i++) {
    sum ^= (p.ch[i] & 0xFF);
    sum ^= (p.ch[i] >> 8);
  }
  sum ^= (p.txID & 0xFF);
  sum ^= (p.txID >> 8);
  sum ^= (p.txID >> 16);
  sum ^= (p.txID >> 24);
  sum ^= p.hopIdx;
  return sum;
}

// --- PPM READ ---
void readPPM() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) pulseIdx = 0;
  else if (pulseIdx < CHANNELS) {
    ppmIn[pulseIdx] = constrain(diff, 1000, 2000);
    pulseIdx++;
  }
}

// --- HOP ---
void generateHopTable(uint32_t seed) {
  randomSeed(seed);
  for (int i = 0; i < HOP_COUNT; i++) {
    hopTable[i] = random(10, 110);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PPM_IN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_IN_PIN), readPPM, RISING);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);

  generateHopTable(transmitterID);

  // --- BIND ---
  radio.setChannel(90);
  radio.openWritingPipe(0xE8E8F0F0E1LL);

  Serial.println("Binding...");
  unsigned long t = millis();
  while (millis() - t < 3000) {
    radio.write(&transmitterID, sizeof(transmitterID));
    delay(5);
  }

  Serial.println("TX Ready");
}

void loop() {
  // --- 100Hz TX ---
  if (micros() - lastSendTime >= 10000) {
    lastSendTime = micros();

    radio.setChannel(hopTable[currentHop]);

    noInterrupts();
    for (int i = 0; i < CHANNELS; i++)
      data.ch[i] = ppmIn[i];
    interrupts();

    data.txID = transmitterID;
    data.hopIdx = currentHop;
    data.crc = calcCRC(data);

    radio.write(&data, sizeof(data));

    currentHop = (currentHop + 1) % HOP_COUNT;
  }

#if DEBUG_PPM
  if (millis() - lastDebugTime > 200) {
    lastDebugTime = millis();

    Serial.print("PPM: ");
    for (int i = 0; i < CHANNELS; i++) {
      Serial.print(ppmIn[i]);
      Serial.print(",");
    }
    Serial.print(" HOP:");
    Serial.print(currentHop);
    Serial.println();
  }
#endif
}