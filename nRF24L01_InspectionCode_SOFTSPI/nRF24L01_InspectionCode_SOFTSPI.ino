#include <RF24.h>
#include <printf.h>
#include <DigitalIO.h>

/* Has to be modified within ~\Documents\Arduino\libraries\RF24\RF24_config.h
#define SOFTSPI     // Requires library from https://github.com/greiman/DigitalIO

#define SOFT_SPI_MISO_PIN A0
#define SOFT_SPI_MOSI_PIN 3
#define SOFT_SPI_SCK_PIN 4
*/

#define CE_PIN 5
#define CSN_PIN A1

RF24 radio(CE_PIN, CSN_PIN);

byte addresses[][6] = {"1Node", "2Node"};


void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); 
  radio.startListening();
  
  Serial.begin(57600);
  printf_begin();

  radio.printDetails();
  
}

void loop() {
//  empty

}
