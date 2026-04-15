#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// --- CONFIGURATION ---
#define SIG_PIN 2
#define CHANNELS 6
#define FRAME_LEN 22500
#define PULSE_LEN 300

RF24 radio(9, 10);

volatile uint16_t ppmOut[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
const byte BIND_PIPE[6] = "BND01";
const byte DATA_PIPE[6] = "DAT01";
uint8_t pnTable[] = {22, 44, 66, 88, 33, 55};
uint8_t pnIndex = 0;
bool isBound = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("--- RX 6-CH DEBUG START ---"));

  if (!radio.begin()) {
    Serial.println(F("Radio Hardware Not Found!"));
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  
  // Bind Mode
  radio.setChannel(100);
  radio.openReadingPipe(1, BIND_PIPE);
  radio.startListening();

  Serial.println(F("Waiting for Bind..."));
  while (!isBound) {
    if (radio.available()) {
      char msg[32] = "";
      radio.read(&msg, sizeof(msg));
      isBound = true;
    }
  }

  Serial.println(F("Bound! Monitoring 6 Channels:"));
  radio.stopListening();
  radio.openReadingPipe(1, DATA_PIPE);
  radio.startListening();

  // Timer1: High-Precision PPM Generator
  pinMode(SIG_PIN, OUTPUT);
  cli();
  TCCR1A = 0; TCCR1B = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Prescaler 8
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = 1000; 
  sei();
}

void loop() {
  radio.setChannel(pnTable[pnIndex]);
  
  unsigned long startWait = millis();
  bool received = false;

  while (millis() - startWait < 22) {
    if (radio.available()) {
      uint16_t incoming[CHANNELS];
      radio.read(&incoming, sizeof(incoming));
      
      // Update PPM values
      for (byte i = 0; i < CHANNELS; i++) {
        ppmOut[i] = incoming[i];
      }
      received = true;
      break;
    }
  }

  // --- 6 CHANNEL DEBUG OUTPUT ---
  if (received) {
    Serial.print(F("CHs ["));
    for (byte i = 0; i < CHANNELS; i++) {
      Serial.print(ppmOut[i]);
      if (i < CHANNELS - 1) Serial.print(F(", "));
    }
    Serial.print(F("] Hop:")); 
    Serial.println(pnTable[pnIndex]);
  } else {
    Serial.print(F("MISSING PACKET on Ch: "));
    Serial.println(pnTable[pnIndex]);
  }

  pnIndex = (pnIndex + 1) % 6;
}

// ISR: Fixed Pulse Generation (CTC Mode)
ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  if (state) {
    digitalWrite(SIG_PIN, LOW);
    OCR1A = PULSE_LEN * 2; 
    state = false;
  } else {
    static byte cur_chan;
    static unsigned int calc_rest;
    digitalWrite(SIG_PIN, HIGH);
    state = true;
    if (cur_chan >= CHANNELS) {
      cur_chan = 0;
      OCR1A = (FRAME_LEN - calc_rest) * 2;
      calc_rest = 0;
    } else {
      OCR1A = (ppmOut[cur_chan] - PULSE_LEN) * 2;
      calc_rest += ppmOut[cur_chan];
      cur_chan++;
    }
  }
}