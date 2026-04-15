#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define SIG_PIN 2
#define CHANNELS 6
#define FRAME_LEN 22500
#define PULSE_LEN 300

RF24 radio(9, 10);

volatile uint16_t ppmOut[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
const byte BIND_PIPE[6] = "BND01";
const byte DATA_PIPE[6] = "DAT01";
uint8_t pnTable[] = {22, 44, 66, 88, 33, 55};
uint8_t pnIdx = 0;
bool isBound = false;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  
  // PHASE 1: BINDING
  radio.setChannel(100);
  radio.openReadingPipe(1, BIND_PIPE);
  radio.startListening();

  while (!isBound) {
    if (radio.available()) {
      char msg[32];
      radio.read(&msg, sizeof(msg));
      isBound = true;
    }
  }

  // PHASE 2: OPERATION
  radio.stopListening();
  radio.openReadingPipe(1, DATA_PIPE);
  radio.startListening();

  // Setup Timer1 for PPM Output
  pinMode(SIG_PIN, OUTPUT);
  cli();
  TCCR1A = 0; TCCR1B = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC Mode, Prescaler 8
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 1000;
  sei();
}

void loop() {
  radio.setChannel(pnTable[pnIdx]);
  
  unsigned long startWait = millis();
  bool received = false;

  while (millis() - startWait < 22) {
    if (radio.available()) {
      uint16_t incoming[CHANNELS];
      radio.read(&incoming, sizeof(incoming));
      for (byte i = 0; i < CHANNELS; i++) ppmOut[i] = incoming[i];
      received = true;
      break;
    }
  }

  // --- CLEAN DATA OUTPUT ---
  if (received) {
    for (byte i = 0; i < CHANNELS; i++) {
      Serial.print(ppmOut[i]);
      if (i < CHANNELS - 1) Serial.print(F(","));
    }
    Serial.println();
  }

  pnIdx = (pnIdx + 1) % 6;
}

// Timer1 ISR for Clean PPM Signal
ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  if (state) {
    digitalWrite(SIG_PIN, LOW);
    OCR1A = PULSE_LEN * 2;
    state = false;
  } else {
    static byte c;
    static unsigned int r;
    digitalWrite(SIG_PIN, HIGH);
    state = true;
    if (c >= CHANNELS) {
      c = 0;
      OCR1A = (FRAME_LEN - r) * 2;
      r = 0;
    } else {
      OCR1A = (ppmOut[c] - PULSE_LEN) * 2;
      r += ppmOut[c];
      c++;
    }
  }
}