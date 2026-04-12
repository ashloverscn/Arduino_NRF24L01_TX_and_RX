#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

// ##########################################
// #####   MultiProtocol nRF24L01 Tx      ######
// ##########################################
// #        by goebish on rcgroups          #
// #                                        #
// #   Parts of this project are derived    #
// #     from existing work, thanks to:     #
// #                                        #
// #   - PhracturedBlue for DeviationTX     #
// #   - victzh for XN297 emulation layer   #
// #   - Hasi for Arduino PPM decoder       #
// #   - hexfet, midelic, closedsink ...    #
// ##########################################

// ############ Wiring ################
#define PPM_pin   2  // PPM in

//SPI Comm.pins with nRF24L01
#define CE_pin    5  // CE   - D5
#define CS_pin    A1 // CS   - A1
#define SCK_pin   4  // SCK  - D4
#define MOSI_pin  3  // MOSI - D3
#define MISO_pin  A0 // MISO - A0

#define ledPin    13 // LED  - D13

/*
//SPI Comm.pins with nRF24L01
#define CE_pin    9 
#define CS_pin    10  
#define SCK_pin   13
#define MOSI_pin  11  
#define MISO_pin  12 

#define ledPin    8 // RXLED  - D8
*/

// SPI outputs
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0

/*
// SPI outputs
#define CE_on PORTB |= _BV(1)   
#define CE_off PORTB &= ~_BV(1) 
#define CS_on PORTB |= _BV(2)   
#define CS_off PORTB &= ~_BV(2) 
#define SCK_on PORTB |= _BV(5)   
#define SCK_off PORTB &= ~_BV(5) 
#define MOSI_on PORTB |= _BV(3)  
#define MOSI_off PORTB &= ~_BV(3)
// SPI input
#define  MISO_on (PINB & _BV(4)) 
*/

#define RF_POWER TX_POWER_158mW 

// PPM stream settings
#define CHANNELS 12 
enum chan_order{
    AILERON, ELEVATOR, THROTTLE, RUDDER,
    AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, AUX8
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// Cabell V3 Constants
#define CABELL_BIND_COUNT     2000
#define CABELL_PACKET_PERIOD  3000
#define CABELL_RADIO_MIN_CHANNEL_NUM 3

// supported protocols
enum {
    PROTO_V2X2 = 0, PROTO_CG023, PROTO_CX10_BLUE, PROTO_CX10_GREEN,
    PROTO_H7, PROTO_BAYANG, PROTO_SYMAX5C1, PROTO_YD829,
    PROTO_H8_3D, PROTO_MJX, PROTO_SYMAXOLD, PROTO_HISKY,
    PROTO_KN, PROTO_YD717, PROTO_FQ777124, PROTO_E010,
    PROTO_BAYANG_SILVERWARE, 
    PROTO_CABELL, 
    PROTO_END
};

enum { ee_PROTOCOL_ID = 0, ee_TXID0, ee_TXID1, ee_TXID2, ee_TXID3 };

struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t updated;
    uint32_t lastUpdate;
} telemetry_data;

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID};

// Cabell Globals
uint16_t bind_counter;
uint8_t hopping_frequency[9];
uint8_t rf_ch_num;
uint8_t packet_count;

void setup() {
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    pinMode(PPM_pin, INPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
    TCCR1A = 0; TCCR1B = 0;
    TCCR1B |= (1 << CS11); 
    set_txid(false);
}

void loop() {
    uint32_t timeout=0;
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    telemetry_data.updated = 0;

    switch(current_protocol) {
        case PROTO_CABELL: timeout = micros() + CABELL_callback(); break;
        case PROTO_CG023: case PROTO_YD829: timeout = process_CG023(); break;
        case PROTO_V2X2: timeout = process_V2x2(); break;
        case PROTO_CX10_GREEN: case PROTO_CX10_BLUE: timeout = process_CX10(); break;
        case PROTO_H7: timeout = process_H7(); break;
        case PROTO_BAYANG: case PROTO_BAYANG_SILVERWARE: timeout = process_Bayang(); break;
        case PROTO_SYMAX5C1: case PROTO_SYMAXOLD: timeout = process_SymaX(); break;
        case PROTO_H8_3D: timeout = process_H8_3D(); break;
        case PROTO_MJX: case PROTO_E010: timeout = process_MJX(); break;
        case PROTO_HISKY: timeout = process_HiSky(); break;
        case PROTO_KN: timeout = process_KN(); break;
        case PROTO_YD717: timeout = process_YD717(); break;
        case PROTO_FQ777124: timeout = process_FQ777124(); break;
    }
    update_ppm();
    while(micros() < timeout);
}

void selectProtocol() {
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok); 
        update_ppm();
        if(ppm[AUX8] < PPM_MAX_COMMAND) count--;
        ppm_ok = false;
    }

    // --- CABELL ASSIGNMENT: RUDDER LEFT + ELEVATOR UP ---
    if(ppm[RUDDER] < PPM_MIN_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_CABELL;
    else if(ppm[RUDDER] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_BAYANG_SILVERWARE;
    else if(ppm[RUDDER] < PPM_MIN_COMMAND) 
        set_txid(true);
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_E010;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_FQ777124;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD717;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_KN;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_HISKY;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_SYMAXOLD;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_MJX;
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H8_3D;
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD829;
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_SYMAX5C1;
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_H7;
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_V2X2;
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_CG023;
    else if(ppm[AILERON] > PPM_MAX_COMMAND)  
        current_protocol = PROTO_CX10_BLUE;
    else if(ppm[AILERON] < PPM_MIN_COMMAND)  
        current_protocol = PROTO_CX10_GREEN;
    else 
        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);      

    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) { delay(100); update_ppm(); }
}

// CABELL V3 IMPLEMENTATION
void CABELL_getChannelSequence(uint8_t outArray[], uint8_t numChannels, uint64_t permutation) {
    uint32_t numChannelsFactorial=1;
    for (uint8_t i = 1; i <= numChannels; i++) {
        numChannelsFactorial *= i;
        outArray[i-1] = i-1;
    }
    uint32_t perm32 = (permutation >> 8);
    perm32 = (perm32 % numChannelsFactorial) << 8;
    perm32 += permutation & 0xFF;
    perm32 = (perm32 % numChannelsFactorial) + 1;
    for (uint8_t i=0, p=perm32-1; i<numChannels; i++ ) {
        numChannelsFactorial /= numChannels-i;
        uint32_t idx = i+(p/numChannelsFactorial);
        p %= numChannelsFactorial;
        uint8_t val = outArray[idx];
        for( ; idx > i; idx--) outArray[idx] = outArray[idx-1];
        outArray[i] = val;
    }
}

uint8_t CABELL_getNextChannel(uint8_t prev) {
    prev -= CABELL_RADIO_MIN_CHANNEL_NUM;
    uint8_t band = (prev / 9 + 3) % 5;
    uint8_t seqIdx = 0;
    for(int x=0; x<9; x++) if(hopping_frequency[x] == (prev % 9)) seqIdx = (x+1)%9;
    return (9 * band) + hopping_frequency[seqIdx] + CABELL_RADIO_MIN_CHANNEL_NUM;
}

void CABELL_send_packet(bool bind) {
    uint8_t pkt[32] = {0};
    rf_ch_num = CABELL_getNextChannel(rf_ch_num);
    pkt[0] = (bind ? 1 : 0) | (packet_count++ << 7);
    uint16_t cs = (rf_ch_num & 0x3F);
    int pIdx = 6;
    for(int i=0; i<16; i++) {
        uint16_t val = (i < 12) ? ppm[i] : 1500;
        if(bind && i > 10) val = 1000 + transmitterID[i-11];
        if(bind && i == THROTTLE) val = 1000;
        if(i % 2) { pkt[pIdx-1] |= (val & 0x0F) << 4; pkt[pIdx++] = (val >> 4) & 0xFF; }
        else { pkt[pIdx++] = val & 0xFF; pkt[pIdx++] = (val >> 8) & 0x0F; }
    }
    for(int i=6; i<30; i++) cs += pkt[i];
    pkt[4] = cs & 0xFF; pkt[5] = cs >> 8;
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);
    NRF24L01_WritePayload(pkt, 32);
}

uint16_t CABELL_callback() {
    CABELL_send_packet(bind_counter > 0);
    if(bind_counter > 0) bind_counter--;
    return CABELL_PACKET_PERIOD;
}

void CABELL_init() {
    bind_counter = CABELL_BIND_COUNT;
    NRF24L01_Initialize();
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
    uint64_t addr = 0;
    for(int i=0; i<4; i++) addr |= ((uint64_t)transmitterID[i] << (8*i));
    CABELL_getChannelSequence(hopping_frequency, 9, addr);
    rf_ch_num = CABELL_RADIO_MIN_CHANNEL_NUM;
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)&addr, 5);
    NRF24L01_SetTxRxMode(TX_EN);
}

void set_txid(bool renew) {
    for(uint8_t i=0; i<4; i++) transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF)) {
        for(uint8_t i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void init_protocol() {
    switch(current_protocol) {
        case PROTO_CABELL: CABELL_init(); break;
        case PROTO_CG023: case PROTO_YD829: CG023_init(); CG023_bind(); break;
        case PROTO_V2X2: V2x2_init(); V2x2_bind(); break;
        case PROTO_CX10_GREEN: case PROTO_CX10_BLUE: CX10_init(); CX10_bind(); break;
        case PROTO_H7: H7_init(); H7_bind(); break;
        case PROTO_BAYANG: case PROTO_BAYANG_SILVERWARE: Bayang_init(); Bayang_bind(); break;
        case PROTO_SYMAX5C1: case PROTO_SYMAXOLD: Symax_init(); break;
        case PROTO_H8_3D: H8_3D_init(); H8_3D_bind(); break;
        case PROTO_MJX: case PROTO_E010: MJX_init(); MJX_bind(); break;
        case PROTO_HISKY: HiSky_init(); break;
        case PROTO_KN: kn_start_tx(true); break;
        case PROTO_YD717: YD717_init(); break;
        case PROTO_FQ777124: FQ777124_init(); FQ777124_bind(); break;
    }
}

void update_ppm() {
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ppm[ch] = Servo_data[ch]; }
    }
}

void ISR_ppm() {
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    if(counterPPM < 1020) pulse = counterPPM;
    else if(counterPPM > 3820) chan = 0;
    else {
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> 1, PPM_MIN, PPM_MAX);
            if(chan==3) ppm_ok = true;
        }
        chan++;
    }
}
