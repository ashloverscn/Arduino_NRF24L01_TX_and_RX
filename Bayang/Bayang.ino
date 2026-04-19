#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

// ===== CONFIG =====
#define PPM_PIN 3
#define CHANNELS 6
#define PACKET_SIZE 15

// ===== GLOBALS =====
uint8_t rx_id[5];
uint8_t hop_table[4];
uint8_t hop_index = 0;

uint16_t channels[CHANNELS];
uint32_t lastPacketTime = 0;

bool bound = false;

// ===== PPM =====
volatile uint16_t ppm[CHANNELS + 1];

void setupPPM() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, 1);

  // Timer1 setup
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, prescaler 8
  OCR1A = 300;
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  static uint8_t ch = 0;
  static uint16_t pulse = 0;

  digitalWrite(PPM_PIN, 0);
  delayMicroseconds(300);
  digitalWrite(PPM_PIN, 1);

  if (ch >= CHANNELS) {
    OCR1A = 22500; // sync
    ch = 0;
  } else {
    OCR1A = ppm[ch] * 2;
    ch++;
  }
}

// ===== RADIO =====
void setupRadio() {
  radio.begin();
  radio.setAutoAck(false);
  radio.disableCRC();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAddressWidth(5);
  radio.openReadingPipe(0, rx_id);
  radio.startListening();
}

// ===== BIND =====
void bind() {
  Serial.println("Binding...");

  uint8_t packet[PACKET_SIZE];

  while (!bound) {
    for (int ch = 0; ch < 126; ch++) {
      radio.setChannel(ch);

      if (radio.available()) {
        radio.read(packet, PACKET_SIZE);

        // Simple Bayang bind detection
        if (packet[0] == 0xA4 || packet[0] == 0xA2) {
          memcpy(rx_id, &packet[1], 5);

          hop_table[0] = packet[6];
          hop_table[1] = packet[7];
          hop_table[2] = packet[8];
          hop_table[3] = packet[9];

          bound = true;

          Serial.println("BOUND!");
          return;
        }
      }
    }
  }
}

// ===== CHANNEL DECODE =====
void decode(uint8_t *p) {
  channels[0] = p[4]; // Throttle
  channels[1] = p[5]; // Yaw
  channels[2] = p[6]; // Pitch
  channels[3] = p[7]; // Roll
  channels[4] = p[8]; // AUX1
  channels[5] = p[9]; // AUX2

  for (int i = 0; i < CHANNELS; i++) {
    ppm[i] = map(channels[i], 0, 255, 1000, 2000);
  }
}

// ===== FHSS =====
void hop() {
  hop_index = (hop_index + 1) % 4;
  radio.setChannel(hop_table[hop_index]);
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  memset(rx_id, 0xAA, 5);

  setupPPM();
  setupRadio();
  bind();

  radio.openReadingPipe(0, rx_id);
}

// ===== LOOP =====
void loop() {
  uint8_t packet[PACKET_SIZE];

  if (radio.available()) {
    radio.read(packet, PACKET_SIZE);

    // basic validation
    if (packet[0] == 0xA5 || packet[0] == 0xA6) {
      decode(packet);
      lastPacketTime = millis();
    }

    hop();
  }

  // FAILSAFE
  if (millis() - lastPacketTime > 500) {
    for (int i = 0; i < CHANNELS; i++) {
      ppm[i] = 1500;
    }
  }
}
