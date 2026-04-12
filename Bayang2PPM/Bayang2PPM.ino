/*
 ##########################################
 #####      Bayang Tx using arduino +       ######
 #####      nRF24L01 to get PPM output      ######
 ##########################################
 #              by 0zoon0                   #
 #                                          #
 #      Most of this project is derived      #
 #       from bikemike/nrf24_multipro.      #
 #    The PPM code got from                 #
 #http://forum.arduino.cc/index.php?topic=309672.0#
 ##########################################
 */

#include "iface_nrf24l01.h"

// uncomment to get some debug info
#define DEBUG

// ############ Wiring ################
#define CE_pin    9 
#define CS_pin    10  
#define SCK_pin   13
#define MOSI_pin  11  
#define MISO_pin  12 

// SPI outputs
#define CE_on PORTB |= _BV(1)   
#define CE_off PORTB &= ~_BV(1) 
#define CS_on PORTB |= _BV(2)   
#define CS_off PORTB &= ~_BV(2) 
#define SCK_on PORTB |= _BV(5)   
#define SCK_off PORTB &= ~_BV(5) 
#define MOSI_on PORTB |= _BV(3)  
#define MOSI_off PORTB &= ~_BV(3)
#define MISO_on (PINB & _BV(4)) 

#define RF_POWER TX_POWER_158mW 

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 6  
#define sigPin 2  
#define PPM_FrLen 27000  
#define PPM_PulseLen 400  
//////////////////////////////////////////////////////////////////

uint8_t transmitterID[4];
uint8_t packet[32];
static uint8_t emptyPacketsCount = 260;
int ppm[channel_number];

struct MyData {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  byte aux1;
};

MyData data;
MyData tmp;

unsigned long lastDebugTime = 0; // For serial throttling

void setup()
{
    pinMode(CE_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

#ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Starting Radio...");
#endif

    resetData();
    setupPPM();
    delay(50);
    SPI_Begin(); 
    NRF24L01_Reset();
    NRF24L01_Initialize();
    Bayang_init();
    Bayang_bind();
}

void loop()
{
    // process protocol
    Bayang_recv_packet(&tmp);
    
    if (tmp.throttle == 0 && tmp.yaw == 0 && tmp.pitch == 0 && tmp.roll == 0) {
      emptyPacketsCount++;
      if (emptyPacketsCount >= 260) {
        resetData(); 
        emptyPacketsCount = 260;
      }
    }
    else {
      data = tmp;
      emptyPacketsCount = 0;
      setPPMValuesFromData();

      // DEBUG OUTPUT
      #ifdef DEBUG
      if (millis() - lastDebugTime > 100) { // Print every 100ms
        Serial.print("CH1:"); Serial.print(ppm[0]); Serial.print("\t");
        Serial.print("CH2:"); Serial.print(ppm[1]); Serial.print("\t");
        Serial.print("CH3:"); Serial.print(ppm[2]); Serial.print("\t");
        Serial.print("CH4:"); Serial.print(ppm[3]); Serial.print("\t");
        Serial.print("CH5:"); Serial.print(ppm[4]); Serial.print("\t");
        Serial.print("CH6:"); Serial.println(ppm[5]);
        lastDebugTime = millis();
      }
      #endif
    }
}

void resetData() 
{
  data.throttle = 0;
  data.yaw = 511;
  data.pitch = 511;
  data.roll = 511;
  data.aux1 = 0;
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.roll,     0, 1023, 1000, 2000);  
  ppm[1] = map(data.pitch,    0, 1023, 1000, 2000);
  ppm[3] = map(data.yaw,      0, 1023, 1000, 2000);
  ppm[2] = map(data.throttle, 0, 1023, 1000, 2000);
  ppm[4] = map(data.aux1,     0, 1,    1000, 2000);
  ppm[5] = 1000; // Fixed 6th channel
}

void setupPPM() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0); 

  cli();
  TCCR1A = 0; 
  TCCR1B = 0;
  OCR1A = 100;  
  TCCR1B |= (1 << WGM12);  
  TCCR1B |= (1 << CS11);  
  TIMSK1 |= (1 << OCIE1A); 
  sei();
}

#define clockMultiplier 2 

ISR(TIMER1_COMPA_vect){
  static boolean state = true;
  TCNT1 = 0;

  if ( state ) {
    PORTD = PORTD & ~B00000100; 
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; 
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
