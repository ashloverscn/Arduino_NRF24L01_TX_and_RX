#include <util/atomic.h>
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

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    ch1,
    ch2,
    ch3,
    ch4,
    ch5,
    ch6,
    ch7,
    ch8,
    ch9,
    ch10,
    ch11,
    ch12,
    
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

static volatile bool ppm_ok = false;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};


void setup()
{
  Serial.begin(115200);
  pinMode(PPM_pin, INPUT);
  // PPM ISR setup
  attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
  TCCR1A = 0;  //reset timer1
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz
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
  update_ppm();
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.
  data.ch1 = mapJoystickValues(ppm[ch1], 1000, 1500, 2000, false);
  data.ch2 = mapJoystickValues(ppm[ch2], 1000, 1500, 2000, false);
  data.ch3 = mapJoystickValues(ppm[ch3], 1000, 1500, 2000, false);
  data.ch4 = mapJoystickValues(ppm[ch4], 1000, 1500, 2000, false);
  data.ch5 = mapJoystickValues(ppm[ch5], 1000, 1500, 2000, false);
  data.ch6 = mapJoystickValues(ppm[ch6], 1000, 1500, 2000, false);
  data.ch7 = mapJoystickValues(ppm[ch7], 1000, 1500, 2000, false);
  data.ch8 = mapJoystickValues(ppm[ch8], 1000, 1500, 2000, false);
  data.ch9 = mapJoystickValues(ppm[ch9], 1000, 1500, 2000, false);
  data.ch10 = mapJoystickValues(ppm[ch10], 1000, 1500, 2000, false);
  data.ch11 = mapJoystickValues(ppm[ch11], 1000, 1500, 2000, false);
  data.ch12 = mapJoystickValues(ppm[ch12], 1000, 1500, 2000, false);

  radio.write(&data, sizeof(MyData));

  /*for(uint8_t i=0; i<CHANNELS; i++)
  {
    Serial.print(ppm[i]);
    Serial.print(',');
  }
  Serial.println();*/
}

void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }
}

void ISR_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}
