
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ############ Wiring ################
#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01
//#define MOSI_pin  11  // MOSI - D3
//#define SCK_pin   13  // SCK  - D4
//#define CE_pin    9  // CE   - D5
//#define MISO_pin  12 // MISO - A0
//#define CS_pin    10 // CS   - A1

/*Create a unique pipe out. The receiver has to 
  wear the same unique code*/
  
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver

RF24 radio(9, 10); // select  CSN  pin

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
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
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.
    
  data.ch1 = 0;
  data.ch2 = 127;
  data.ch3 = 127;
  data.ch4 = 127;
  data.ch5 = 0;
  data.ch6 = 0;
  data.ch7 = 0;
  data.ch8 = 0;
  data.ch9 = 0;
  data.ch10 = 0;
  data.ch11 = 0;
  data.ch12 = 0;
}

void setup()
{
  //Start everything up
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.
  data.ch1 = mapJoystickValues( analogRead(A0), 13, 524, 1015, true );
  data.ch2 = mapJoystickValues( analogRead(A1),  1, 505, 1020, true );
  data.ch3 = mapJoystickValues( analogRead(A2), 12, 544, 1021, true );
  data.ch4 = mapJoystickValues( analogRead(A3), 34, 522, 1020, true );
  data.ch5 = mapJoystickValues( analogRead(A4), 34, 522, 1020, true );
  data.ch6 = mapJoystickValues( analogRead(A5), 34, 522, 1020, true );
  data.ch6 = mapJoystickValues( analogRead(A6), 34, 522, 1020, true );
  data.ch6 = mapJoystickValues( analogRead(A7), 34, 522, 1020, true );
  //data.ch6 = mapJoystickValues( analogRead(A7), 34, 522, 1020, true );
  //data.ch6 = mapJoystickValues( analogRead(A7), 34, 522, 1020, true );
  //data.ch6 = mapJoystickValues( analogRead(A7), 34, 522, 1020, true );
  //data.ch6 = mapJoystickValues( analogRead(A7), 34, 522, 1020, true );

  radio.write(&data, sizeof(MyData));
}
