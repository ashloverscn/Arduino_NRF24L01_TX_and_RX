#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define SIG_PIN 2
#define CHANNELS 6
#define HOP_COUNT 16
#define FRAME_LEN 22500
#define PULSE_LEN 300

RF24 radio(9, 10);

struct Payload {
  uint16_t ch[CHANNELS];
  uint32_t txID;
  uint8_t hopIdx;
};

volatile uint16_t ppmOut[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
uint32_t activeTX_ID = 0;
uint8_t hopTable[HOP_COUNT];
uint8_t currentHop = 0;
unsigned long lastValidTime = 0;

void generateHopTable(uint32_t seed) {
  randomSeed(seed);
  for (int i = 0; i < HOP_COUNT; i++) {
    hopTable[i] = random(2, 124);
  }
}

void setupPPMTimer() {
  cli();
  TCCR1A = 0; 
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC Mode, Prescaler 8
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

  // Binding: Listen for TX ID on Channel 100
  radio.setChannel(100);
  radio.openReadingPipe(1, 0xE8E8F0F0E1LL);
  radio.startListening();

  Serial.println(F("Listening for TX ID..."));
  while (activeTX_ID == 0) {
    if (radio.available()) {
      radio.read(&activeTX_ID, sizeof(activeTX_ID));
      generateHopTable(activeTX_ID);
    }
  }

  Serial.println(F("ID Locked. Starting AFHDS Logic."));
  
  radio.stopListening();
  radio.openReadingPipe(1, 0xE8E8F0F0E1LL); 
  radio.startListening();

  pinMode(SIG_PIN, OUTPUT);
  setupPPMTimer(); 
}

void loop() {
  radio.setChannel(hopTable[currentHop]);
  
  unsigned long window = micros();
  bool found = false;

  // Search window for packet arrival
  while (micros() - window < 12000) {
    if (radio.available()) {
      Payload tmp;
      radio.read(&tmp, sizeof(tmp));
      
      if (tmp.txID == activeTX_ID) {
        for(int i=0; i<CHANNELS; i++) ppmOut[i] = tmp.ch[i];
        currentHop = tmp.hopIdx; // Direct resync from TX packet
        lastValidTime = millis();
        found = true;
        break;
      }
    }
  }

  // --- CLEAN 6-CH DEBUG OUTPUT ---
  // Prints every 16 hops to ensure Serial doesn't lag the timing
  if (found && currentHop == 0) {
    for (byte i = 0; i < CHANNELS; i++) {
      Serial.print(ppmOut[i]);
      if (i < CHANNELS - 1) Serial.print(F(","));
    }
    Serial.println(); 
  }

  // Failsafe: Neutralize if no valid packet for 500ms
  if (millis() - lastValidTime > 500) {
    for(int i=0; i<CHANNELS; i++) ppmOut[i] = 1500;
    ppmOut[2] = 1000; // Low Throttle
  }

  currentHop = (currentHop + 1) % HOP_COUNT;
}

// Timer1 ISR for PPM Generation
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