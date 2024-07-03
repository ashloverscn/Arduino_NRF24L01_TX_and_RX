
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 12  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[channel_number];

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
  byte ch6;
  byte ch7;
  byte ch8;
  byte ch9;
  byte ch10;
  byte ch11;
  byte ch12;
};

MyData data;

void resetData() 
{
  // 'safe' values to use when no radio input is detected
  data.ch1 = 0;
  data.ch2 = 127;
  data.ch3 = 127;
  data.ch4 = 127;
  data.ch5 = 0;
  data.ch6= 0;
  data.ch7= 0;
  data.ch8= 0;
  data.ch9= 0;
  data.ch10= 0;
  data.ch11= 0;
  data.ch12= 0;
  
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.ch1, 0, 255, 1000, 2000);
  ppm[1] = map(data.ch2, 0, 255, 1000, 2000);
  ppm[2] = map(data.ch3, 0, 255, 1000, 2000);
  ppm[3] = map(data.ch4, 0, 255, 1000, 2000);
  ppm[4] = map(data.ch5, 0, 255, 1000, 2000);
  ppm[5] = map(data.ch6, 0, 255, 1000, 2000);  
  ppm[6] = map(data.ch7, 0, 255, 1000, 2000);  
  ppm[7] = map(data.ch8, 0, 255, 1000, 2000);  
  ppm[8] = map(data.ch9, 0, 255, 1000, 2000);  
  ppm[9] = map(data.ch10, 0, 255, 1000, 2000);  
  ppm[10] = map(data.ch11, 0, 255, 1000, 2000);  
  ppm[11] = map(data.ch12, 0, 255, 1000, 2000);  
  }

/**************************************************/

void setupPPM() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void setup()
{  
  resetData();
  setupPPM();
  
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.setAutoAck(false);

  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}

/**************************************************/

void loop()
{
  recvData();

  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  
  setPPMValuesFromData();
}

/**************************************************/

//#error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
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
