#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define SIG_PIN 2
#define CHANNELS 6
#define HOP_COUNT 16
#define FRAME_LEN 22500
#define PULSE_LEN 300
#define DEBUG 1

RF24 radio(9, 10);

struct Payload {
  uint16_t ch[CHANNELS];
  uint32_t txID;
  uint8_t hopIdx;
  uint8_t crc;
};

volatile uint16_t ppmOut[CHANNELS] = {1500,1500,1500,1500,1500,1500};

uint32_t activeTX_ID = 0;
uint8_t hopTable[HOP_COUNT];
uint8_t currentHop = 0;

unsigned long lastValidTime = 0;
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

// --- HOP ---
void generateHopTable(uint32_t seed) {
  randomSeed(seed);
  for (int i = 0; i < HOP_COUNT; i++) {
    hopTable[i] = random(10, 110);
  }
}

// --- TIMER ---
void setupPPMTimer() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 1000;
  sei();
}

void setup() {
  Serial.begin(115200);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);

  // --- BIND ---
  radio.setChannel(90);
  radio.openReadingPipe(1, 0xE8E8F0F0E1LL);
  radio.startListening();

  Serial.println("Waiting bind...");

  while (activeTX_ID == 0) {
    if (radio.available()) {
      radio.read(&activeTX_ID, sizeof(activeTX_ID));
      generateHopTable(activeTX_ID);
    }
  }

  Serial.println("Bound!");

  pinMode(SIG_PIN, OUTPUT);
  setupPPMTimer();
}

void loop() {
  radio.setChannel(hopTable[currentHop]);

  unsigned long start = micros();

  while (micros() - start < 10000) {
    if (radio.available()) {
      Payload p;
      radio.read(&p, sizeof(p));

      if (p.txID == activeTX_ID && p.crc == calcCRC(p)) {
        for (int i = 0; i < CHANNELS; i++)
          ppmOut[i] = constrain(p.ch[i], 1000, 2000);

        currentHop = (p.hopIdx + 1) % HOP_COUNT;
        lastValidTime = millis();
        break;
      }
    }
  }

  // --- FAILSAFE ---
  if (millis() - lastValidTime > 500) {
    for (int i = 0; i < CHANNELS; i++) ppmOut[i] = 1500;
    ppmOut[2] = 1000;
  }

#if DEBUG
  if (millis() - lastDebugTime > 200) {
    lastDebugTime = millis();

    Serial.print("CH: ");
    for (int i = 0; i < CHANNELS; i++) {
      Serial.print(ppmOut[i]);
      Serial.print(",");
    }
    Serial.print(" HOP:");
    Serial.print(currentHop);
    Serial.println();
  }
#endif

  currentHop = (currentHop + 1) % HOP_COUNT;
}

// --- PPM OUTPUT ---
ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  static byte ch = 0;
  static unsigned int sum = 0;

  if (state) {
    digitalWrite(SIG_PIN, LOW);
    OCR1A = PULSE_LEN * 2;
    state = false;
  } else {
    digitalWrite(SIG_PIN, HIGH);
    state = true;

    if (ch >= CHANNELS) {
      ch = 0;
      OCR1A = (FRAME_LEN - sum) * 2;
      sum = 0;
    } else {
      OCR1A = (ppmOut[ch] - PULSE_LEN) * 2;
      sum += ppmOut[ch];
      ch++;
    }
  }
}