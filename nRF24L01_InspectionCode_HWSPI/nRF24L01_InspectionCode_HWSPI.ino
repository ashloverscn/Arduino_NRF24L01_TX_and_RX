#include <SPI.h>
#include <RF24.h>
#include <printf.h>

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

byte addresses[][6] = {"1Node", "2Node"};


void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]); 
  radio.startListening();
  
  Serial.begin(115200);
  printf_begin();

  radio.printDetails();
  
}

void loop() {
//  empty

}
