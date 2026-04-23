#include <SPI.h>

#define CE_PIN   9
#define CSN_PIN  10

void csn_low()  { digitalWrite(CSN_PIN, LOW); }
void csn_high() { digitalWrite(CSN_PIN, HIGH); }
void ce_high()  { digitalWrite(CE_PIN, HIGH); }
void ce_low()   { digitalWrite(CE_PIN, LOW); }

uint8_t spi_transfer(uint8_t data) {
  return SPI.transfer(data);
}

void setup() {
  Serial.begin(115200);

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);

  csn_high();
  ce_low();

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  Serial.println("SPI_READY");
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {

    // Single transfer: T <byte>
    case 'T': {
      while (!Serial.available());
      uint8_t val = Serial.read();
      uint8_t resp = spi_transfer(val);
      Serial.write(resp);
      break;
    }

    // Burst transfer: B <len> <bytes...>
    case 'B': {
      while (Serial.available() < 1);
      uint8_t len = Serial.read();

      for (uint8_t i = 0; i < len; i++) {
        while (!Serial.available());
        uint8_t d = Serial.read();
        uint8_t r = spi_transfer(d);
        Serial.write(r);
      }
      break;
    }

    // CSN LOW
    case 'c':
      csn_low();
      break;

    // CSN HIGH
    case 'C':
      csn_high();
      break;

    // CE HIGH
    case 'e':
      ce_high();
      break;

    // CE LOW
    case 'E':
      ce_low();
      break;

    default:
      break;
  }
}