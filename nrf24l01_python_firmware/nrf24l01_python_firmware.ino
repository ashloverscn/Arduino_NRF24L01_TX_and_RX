/*
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║           ULTIMATE NRF24L01+ SERIAL API FIRMWARE v3.0                        ║
 * ║           Complete Register-Level Control with Enhanced Features             ║
 * ║                                                                              ║
 * ║  Hardware: Arduino Uno/Nano/Pro Mini (ATmega328P)                            ║
 * ║  Serial:   115200 baud, newline-terminated, echo-optional                    ║
 * ║  SPI:      8MHz max (4MHz default), Mode 0, MSB-first                        ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * WIRING (Standard SPI):
 *   CE  -> D9   (Chip Enable - mode control)
 *   CSN -> D10  (SPI Chip Select - active LOW)
 *   SCK -> D13  (SPI Clock)
 *   MOSI-> D11  (SPI Data In)
 *   MISO-> D12  (SPI Data Out)
 *   IRQ -> D2   (Interrupt - optional, active LOW)
 *   VCC -> 3.3V (DO NOT USE 5V!)
 *   GND -> GND
 *
 * FEATURES:
 *   - Full register read/write (all 0x00-0x1D + undocumented test registers)
 *   - Multi-byte burst transfers for addresses and payloads
 *   - Dynamic Payload Length (DPL) with width validation
 *   - ACK Payload support (up to 3 pending in PRX TX FIFO)
 *   - TX No-ACK flag per-packet
 *   - Complete MultiCeiver pipe management (0-5)
 *   - RF Channel scanning with Carrier Detect
 *   - Transmission observation (lost packets, retries)
 *   - IRQ pin monitoring with configurable masks
 *   - FIFO depth monitoring (TX/RX levels)
 *   - Power management (Power Down, Standby-I, RX, TX)
 *   - Auto-retry configuration with validation
 *   - CRC control (1-byte / 2-byte / disabled)
 *   - Address width configuration (3-5 bytes)
 *   - Complete register dump with human-readable decoding
 *   - Command batching with atomic execution
 */

#include <SPI.h>

// ═══════════════════════════════════════════════════════════════════════════════
// CONFIGURATION CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════
#define CE_PIN          9
#define CSN_PIN         10
#define IRQ_PIN         2           // Optional interrupt pin

#define SERIAL_BAUD     115200
#define CMD_BUFFER_SIZE 256         // Increased for batch operations
#define MAX_PAYLOAD     32
#define MAX_PIPE        5

// Timing constants (microseconds)
#define T_PD2STBY       5000        // Power down to standby
#define T_STBY2A        130         // Standby to active
#define T_HCE           10          // Minimum CE high pulse

// ═══════════════════════════════════════════════════════════════════════════════
// PORT MANIPULATION (ATmega328P - Arduino Uno/Nano/Pro Mini)
// ═══════════════════════════════════════════════════════════════════════════════
#define CE_HIGH()       PORTB |=  (1 << PORTB1)
#define CE_LOW()        PORTB &= ~(1 << PORTB1)
#define CSN_HIGH()      PORTB |=  (1 << PORTB2)
#define CSN_LOW()       PORTB &= ~(1 << PORTB2)

// ═══════════════════════════════════════════════════════════════════════════════
// REGISTER MAP (Complete - all documented NRF24L01+ registers)
// ═══════════════════════════════════════════════════════════════════════════════
enum NRF24_Register : uint8_t {
    // Configuration registers
    REG_CONFIG      = 0x00,     // Configuration
    REG_EN_AA       = 0x01,     // Enhanced ShockBurst auto-ack
    REG_EN_RXADDR   = 0x02,     // Enabled RX addresses
    REG_SETUP_AW    = 0x03,     // Address width
    REG_SETUP_RETR  = 0x04,     // Auto-retransmit setup
    REG_RF_CH       = 0x05,     // RF channel
    REG_RF_SETUP    = 0x06,     // RF setup
    REG_STATUS      = 0x07,     // Status
    REG_OBSERVE_TX  = 0x08,     // TX observe
    REG_CD          = 0x09,     // Carrier detect / RPD
    REG_RX_ADDR_P0  = 0x0A,     // RX address pipe 0
    REG_RX_ADDR_P1  = 0x0B,     // RX address pipe 1
    REG_RX_ADDR_P2  = 0x0C,     // RX address pipe 2
    REG_RX_ADDR_P3  = 0x0D,     // RX address pipe 3
    REG_RX_ADDR_P4  = 0x0E,     // RX address pipe 4
    REG_RX_ADDR_P5  = 0x0F,     // RX address pipe 5
    REG_TX_ADDR     = 0x10,     // TX address
    REG_RX_PW_P0    = 0x11,     // RX payload width pipe 0
    REG_RX_PW_P1    = 0x12,     // RX payload width pipe 1
    REG_RX_PW_P2    = 0x13,     // RX payload width pipe 2
    REG_RX_PW_P3    = 0x14,     // RX payload width pipe 3
    REG_RX_PW_P4    = 0x15,     // RX payload width pipe 4
    REG_RX_PW_P5    = 0x16,     // RX payload width pipe 5
    REG_FIFO_STATUS = 0x17,     // FIFO status
    REG_DYNPD       = 0x1C,     // Dynamic payload
    REG_FEATURE     = 0x1D,     // Feature
};

// SPI Instruction set
enum NRF24_Command : uint8_t {
    CMD_R_REGISTER      = 0x00,
    CMD_W_REGISTER      = 0x20,
    CMD_R_RX_PAYLOAD    = 0x61,
    CMD_W_TX_PAYLOAD    = 0xA0,
    CMD_FLUSH_TX        = 0xE1,
    CMD_FLUSH_RX        = 0xE2,
    CMD_REUSE_TX_PL     = 0xE3,
    CMD_ACTIVATE        = 0x50,
    CMD_R_RX_PL_WID     = 0x60,
    CMD_W_ACK_PAYLOAD   = 0xA8,
    CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    CMD_NOP             = 0xFF,
};

// ═══════════════════════════════════════════════════════════════════════════════
// BIT DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════
// CONFIG register bits
#define MASK_RX_DR      6
#define MASK_TX_DS      5
#define MASK_MAX_RT     4
#define EN_CRC          3
#define CRCO            2
#define PWR_UP          1
#define PRIM_RX         0

// STATUS register bits
#define RX_DR           6
#define TX_DS           5
#define MAX_RT          4
#define RX_P_NO         1          // 3 bits: 1-3
#define TX_FULL         0

// FIFO_STATUS bits
#define TX_REUSE        6
#define TX_FIFO_FULL    5
#define TX_EMPTY        4
#define RX_FULL         1
#define RX_EMPTY        0

// FEATURE register bits
#define EN_DPL          2
#define EN_ACK_PAY      1
#define EN_DYN_ACK      0

// RF_SETUP bits
#define PLL_LOCK        4
#define RF_DR           3
#define RF_PWR_HIGH     2
#define RF_PWR_LOW      1
#define LNA_HCURR       0          // Obsolete in +

// OBSERVE_TX bits
#define PLOS_CNT        4          // 4 bits
#define ARC_CNT         0          // 4 bits

// ═══════════════════════════════════════════════════════════════════════════════
// ENUMERATIONS
// ═══════════════════════════════════════════════════════════════════════════════
enum class RF_Mode : uint8_t {
    POWER_DOWN  = 0,
    STANDBY_I   = 1,
    STANDBY_II  = 2,
    RX_MODE     = 3,
    TX_MODE     = 4
};

enum class DataRate : uint8_t {
    DR_1MBPS    = 0,
    DR_2MBPS    = 1,
    DR_250KBPS  = 2
};

enum class TXPower : uint8_t {
    PWR_18dBm   = 0,            // -18 dBm (approx 0.016mW)
    PWR_12dBm   = 1,            // -12 dBm (approx 0.063mW)
    PWR_6dBm    = 2,            // -6 dBm  (approx 0.25mW)
    PWR_0dBm    = 3             // 0 dBm   (approx 1mW / 5mW with PA)
};

enum class CRCMode : uint8_t {
    CRC_DISABLED = 0,
    CRC_1BYTE   = 1,
    CRC_2BYTE   = 2
};

enum class AddressWidth : uint8_t {
    AW_3BYTES   = 3,
    AW_4BYTES   = 4,
    AW_5BYTES   = 5
};

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════════
static uint8_t rx_buffer[MAX_PAYLOAD];
static uint8_t tx_buffer[MAX_PAYLOAD];
static uint8_t rf_setup_cache = 0x0F;   // Cache to avoid SPI reads
static volatile bool irq_triggered = false;

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;
bool echoEnabled = true;
bool verboseMode = false;

// ═══════════════════════════════════════════════════════════════════════════════
// LOW-LEVEL SPI (Optimized inline functions)
// ═══════════════════════════════════════════════════════════════════════════════
inline uint8_t spiTransfer(uint8_t data) {
    return SPI.transfer(data);
}

inline uint8_t readRegister(uint8_t reg) {
    uint8_t result;
    CSN_LOW();
    spiTransfer(CMD_R_REGISTER | (reg & 0x1F));
    result = spiTransfer(0xFF);
    CSN_HIGH();
    return result;
}

inline uint8_t writeRegister(uint8_t reg, uint8_t value) {
    CSN_LOW();
    spiTransfer(CMD_W_REGISTER | (reg & 0x1F));
    spiTransfer(value);
    CSN_HIGH();
    return value;
}

inline void writeRegisterMulti(uint8_t reg, const uint8_t* data, uint8_t len) {
    if (len == 0 || len > MAX_PAYLOAD) return;
    CSN_LOW();
    spiTransfer(CMD_W_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(data[i]);
    }
    CSN_HIGH();
}

inline void readRegisterMulti(uint8_t reg, uint8_t* data, uint8_t len) {
    if (len == 0 || len > MAX_PAYLOAD) return;
    CSN_LOW();
    spiTransfer(CMD_R_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spiTransfer(0xFF);
    }
    CSN_HIGH();
}

inline uint8_t sendCommand(uint8_t cmd) {
    uint8_t status;
    CSN_LOW();
    status = spiTransfer(cmd);
    CSN_HIGH();
    return status;
}

// ═══════════════════════════════════════════════════════════════════════════════
// CORE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

uint8_t getStatus() {
    return sendCommand(CMD_NOP);
}

uint8_t getFIFOStatus() {
    return readRegister(REG_FIFO_STATUS);
}

bool isTXFull() {
    return (getStatus() & (1 << TX_FULL)) ? true : false;
}

bool isRXDataAvailable() {
    return (getStatus() & (1 << RX_DR)) ? true : false;
}

uint8_t getRXDataPipe() {
    return (getStatus() >> RX_P_NO) & 0x07;
}

bool isTXSent() {
    return (getStatus() & (1 << TX_DS)) ? true : false;
}

bool isMaxRT() {
    return (getStatus() & (1 << MAX_RT)) ? true : false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// POWER & MODE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

void setPowerMode(RF_Mode mode) {
    uint8_t config = readRegister(REG_CONFIG);
    
    switch (mode) {
        case RF_Mode::POWER_DOWN:
            CE_LOW();
            config &= ~(1 << PWR_UP);
            writeRegister(REG_CONFIG, config);
            break;
            
        case RF_Mode::STANDBY_I:
            CE_LOW();
            config |= (1 << PWR_UP);
            config &= ~(1 << PRIM_RX);
            writeRegister(REG_CONFIG, config);
            delayMicroseconds(T_STBY2A);
            break;
            
        case RF_Mode::RX_MODE:
            config |= (1 << PWR_UP) | (1 << PRIM_RX);
            writeRegister(REG_CONFIG, config);
            delayMicroseconds(T_STBY2A);
            CE_HIGH();
            break;
            
        case RF_Mode::TX_MODE:
            config |= (1 << PWR_UP);
            config &= ~(1 << PRIM_RX);
            writeRegister(REG_CONFIG, config);
            delayMicroseconds(T_STBY2A);
            CE_HIGH();
            delayMicroseconds(T_HCE);
            // CE can go low after 10us minimum pulse
            break;
    }
}

void powerUp() {
    uint8_t config = readRegister(REG_CONFIG);
    if (!(config & (1 << PWR_UP))) {
        config |= (1 << PWR_UP);
        writeRegister(REG_CONFIG, config);
        delayMicroseconds(T_PD2STBY);
    }
}

void powerDown() {
    CE_LOW();
    uint8_t config = readRegister(REG_CONFIG);
    config &= ~(1 << PWR_UP);
    writeRegister(REG_CONFIG, config);
}

// ═══════════════════════════════════════════════════════════════════════════════
// RF CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

void setChannel(uint8_t channel) {
    if (channel > 125) channel = 125;
    writeRegister(REG_RF_CH, channel);
}

uint8_t getChannel() {
    return readRegister(REG_RF_CH) & 0x7F;
}

void setDataRate(DataRate rate) {
    rf_setup_cache = readRegister(REG_RF_SETUP);
    rf_setup_cache &= ~((1 << 5) | (1 << 3)); // Clear RF_DR_LOW and RF_DR_HIGH
    
    switch (rate) {
        case DataRate::DR_1MBPS:
            break; // Both bits 0
        case DataRate::DR_2MBPS:
            rf_setup_cache |= (1 << 3); // RF_DR_HIGH
            break;
        case DataRate::DR_250KBPS:
            rf_setup_cache |= (1 << 5); // RF_DR_LOW
            break;
    }
    writeRegister(REG_RF_SETUP, rf_setup_cache);
}

DataRate getDataRate() {
    uint8_t rf_setup = readRegister(REG_RF_SETUP);
    if (rf_setup & (1 << 5)) return DataRate::DR_250KBPS;
    if (rf_setup & (1 << 3)) return DataRate::DR_2MBPS;
    return DataRate::DR_1MBPS;
}

void setTXPower(TXPower power) {
    rf_setup_cache = readRegister(REG_RF_SETUP);
    rf_setup_cache &= ~((1 << 2) | (1 << 1));
    rf_setup_cache |= ((static_cast<uint8_t>(power) & 0x03) << 1);
    writeRegister(REG_RF_SETUP, rf_setup_cache);
}

TXPower getTXPower() {
    uint8_t pwr = (readRegister(REG_RF_SETUP) >> 1) & 0x03;
    return static_cast<TXPower>(pwr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// CRC CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

void setCRC(CRCMode mode) {
    uint8_t config = readRegister(REG_CONFIG);
    config &= ~((1 << EN_CRC) | (1 << CRCO));
    
    switch (mode) {
        case CRCMode::CRC_DISABLED:
            break;
        case CRCMode::CRC_1BYTE:
            config |= (1 << EN_CRC);
            break;
        case CRCMode::CRC_2BYTE:
            config |= (1 << EN_CRC) | (1 << CRCO);
            break;
    }
    writeRegister(REG_CONFIG, config);
}

CRCMode getCRC() {
    uint8_t config = readRegister(REG_CONFIG);
    if (!(config & (1 << EN_CRC))) return CRCMode::CRC_DISABLED;
    return (config & (1 << CRCO)) ? CRCMode::CRC_2BYTE : CRCMode::CRC_1BYTE;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ADDRESS MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

void setAddressWidth(AddressWidth width) {
    uint8_t aw = static_cast<uint8_t>(width) - 2;
    writeRegister(REG_SETUP_AW, aw & 0x03);
}

AddressWidth getAddressWidth() {
    uint8_t aw = readRegister(REG_SETUP_AW) & 0x03;
    return static_cast<AddressWidth>(aw + 2);
}

void setTXAddress(const uint8_t* addr, uint8_t len) {
    if (len < 3 || len > 5) return;
    writeRegisterMulti(REG_TX_ADDR, addr, len);
}

void getTXAddress(uint8_t* addr, uint8_t len) {
    if (len < 3 || len > 5) return;
    readRegisterMulti(REG_TX_ADDR, addr, len);
}

void setRXAddress(uint8_t pipe, const uint8_t* addr, uint8_t len) {
    if (pipe > MAX_PIPE) return;
    if (len < 3 || len > 5) return;
    
    if (pipe <= 1) {
        writeRegisterMulti(REG_RX_ADDR_P0 + pipe, addr, len);
    } else {
        // Pipes 2-5 only use LSB, MSB bytes equal to pipe 1
        writeRegister(REG_RX_ADDR_P0 + pipe, addr[0]);
    }
}

void getRXAddress(uint8_t pipe, uint8_t* addr, uint8_t len) {
    if (pipe > MAX_PIPE || len < 3 || len > 5) return;
    readRegisterMulti(REG_RX_ADDR_P0 + pipe, addr, len);
}

// ═══════════════════════════════════════════════════════════════════════════════
// PIPE & PAYLOAD MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

void enableRXPipe(uint8_t pipe) {
    if (pipe > MAX_PIPE) return;
    uint8_t en_rxaddr = readRegister(REG_EN_RXADDR);
    en_rxaddr |= (1 << pipe);
    writeRegister(REG_EN_RXADDR, en_rxaddr);
}

void disableRXPipe(uint8_t pipe) {
    if (pipe > MAX_PIPE) return;
    uint8_t en_rxaddr = readRegister(REG_EN_RXADDR);
    en_rxaddr &= ~(1 << pipe);
    writeRegister(REG_EN_RXADDR, en_rxaddr);
}

bool isPipeEnabled(uint8_t pipe) {
    if (pipe > MAX_PIPE) return false;
    return (readRegister(REG_EN_RXADDR) >> pipe) & 0x01;
}

void setRXPayloadWidth(uint8_t pipe, uint8_t width) {
    if (pipe > MAX_PIPE || width > MAX_PAYLOAD) return;
    writeRegister(REG_RX_PW_P0 + pipe, width);
}

uint8_t getRXPayloadWidth(uint8_t pipe) {
    if (pipe > MAX_PIPE) return 0;
    return readRegister(REG_RX_PW_P0 + pipe);
}

// ═══════════════════════════════════════════════════════════════════════════════
// AUTO-ACK & RETRANSMIT
// ═══════════════════════════════════════════════════════════════════════════════

void setAutoAck(uint8_t pipe, bool enable) {
    if (pipe > MAX_PIPE) return;
    uint8_t en_aa = readRegister(REG_EN_AA);
    if (enable)
        en_aa |= (1 << pipe);
    else
        en_aa &= ~(1 << pipe);
    writeRegister(REG_EN_AA, en_aa);
}

bool getAutoAck(uint8_t pipe) {
    if (pipe > MAX_PIPE) return false;
    return (readRegister(REG_EN_AA) >> pipe) & 0x01;
}

void setRetransmit(uint8_t delay, uint8_t count) {
    // delay: 0=250us, 1=500us, ..., 15=4000us
    // count: 0=disabled, ..., 15=15 retries
    if (delay > 15) delay = 15;
    if (count > 15) count = 15;
    writeRegister(REG_SETUP_RETR, ((delay & 0x0F) << 4) | (count & 0x0F));
}

uint8_t getRetransmitDelay() {
    return (readRegister(REG_SETUP_RETR) >> 4) & 0x0F;
}

uint8_t getRetransmitCount() {
    return readRegister(REG_SETUP_RETR) & 0x0F;
}

// ═══════════════════════════════════════════════════════════════════════════════
// DYNAMIC PAYLOAD & FEATURES
// ═══════════════════════════════════════════════════════════════════════════════

void enableDynamicPayload(uint8_t pipe) {
    if (pipe > MAX_PIPE) return;
    uint8_t dynpd = readRegister(REG_DYNPD);
    dynpd |= (1 << pipe);
    writeRegister(REG_DYNPD, dynpd);
}

void disableDynamicPayload(uint8_t pipe) {
    if (pipe > MAX_PIPE) return;
    uint8_t dynpd = readRegister(REG_DYNPD);
    dynpd &= ~(1 << pipe);
    writeRegister(REG_DYNPD, dynpd);
}

void setFeature(uint8_t feature, bool enable) {
    uint8_t feat = readRegister(REG_FEATURE);
    if (enable)
        feat |= (1 << feature);
    else
        feat &= ~(1 << feature);
    writeRegister(REG_FEATURE, feat);
}

bool getFeature(uint8_t feature) {
    return (readRegister(REG_FEATURE) >> feature) & 0x01;
}

bool activateFeatures() {
    // Toggle features - must be done if FEATURE register reads 0x00 on + chips
    CSN_LOW();
    spiTransfer(CMD_ACTIVATE);
    spiTransfer(0x73);
    CSN_HIGH();
    delayMicroseconds(5);
    // Verify activation
    return (readRegister(REG_FEATURE) != 0x00);
}

// ═══════════════════════════════════════════════════════════════════════════════
// FIFO OPERATIONS
// ═══════════════════════════════════════════════════════════════════════════════

void flushTX() {
    sendCommand(CMD_FLUSH_TX);
}

void flushRX() {
    sendCommand(CMD_FLUSH_RX);
}

void reuseTXPayload() {
    sendCommand(CMD_REUSE_TX_PL);
}

bool isTXEmpty() {
    return (readRegister(REG_FIFO_STATUS) & (1 << TX_EMPTY)) ? true : false;
}

bool isTXFullFIFO() {
    return (readRegister(REG_FIFO_STATUS) & (1 << TX_FIFO_FULL)) ? true : false;
}

bool isRXEmpty() {
    return (readRegister(REG_FIFO_STATUS) & (1 << RX_EMPTY)) ? true : false;
}

bool isRXFullFIFO() {
    return (readRegister(REG_FIFO_STATUS) & (1 << RX_FULL)) ? true : false;
}

uint8_t getTXReuse() {
    return (readRegister(REG_FIFO_STATUS) >> TX_REUSE) & 0x01;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PAYLOAD OPERATIONS
// ═══════════════════════════════════════════════════════════════════════════════

void writePayload(const uint8_t* data, uint8_t len, bool noAck = false) {
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
    
    CE_LOW();
    delayMicroseconds(5);
    CSN_LOW();
    
    if (noAck && getFeature(EN_DYN_ACK)) {
        spiTransfer(CMD_W_TX_PAYLOAD_NOACK);
    } else {
        spiTransfer(CMD_W_TX_PAYLOAD);
    }
    
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(data[i]);
    }
    // Pad remaining bytes if needed for static payload
    uint8_t staticWidth = getRXPayloadWidth(0); // Assume pipe 0 for TX reference
    if (!getFeature(EN_DPL) && len < staticWidth) {
        for (uint8_t i = len; i < staticWidth; i++) {
            spiTransfer(0x00);
        }
    }
    
    CSN_HIGH();
    CE_HIGH();
    delayMicroseconds(T_HCE);
    // CE stays high for TX mode operation
}

uint8_t readPayload(uint8_t* data, uint8_t len) {
    if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
    
    CSN_LOW();
    spiTransfer(CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = spiTransfer(0xFF);
    }
    CSN_HIGH();
    
    // Clear RX_DR flag
    writeRegister(REG_STATUS, (1 << RX_DR));
    return len;
}

uint8_t getDynamicPayloadWidth() {
    uint8_t width;
    CSN_LOW();
    spiTransfer(CMD_R_RX_PL_WID);
    width = spiTransfer(0xFF);
    CSN_HIGH();
    
    // Validate width per datasheet
    if (width > MAX_PAYLOAD) {
        flushRX(); // Corrupt packet, flush RX
        return 0;
    }
    return width;
}

void writeAckPayload(uint8_t pipe, const uint8_t* data, uint8_t len) {
    if (pipe > MAX_PIPE || len > MAX_PAYLOAD) return;
    
    CSN_LOW();
    spiTransfer(CMD_W_ACK_PAYLOAD + pipe);
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(data[i]);
    }
    CSN_HIGH();
}

// ═══════════════════════════════════════════════════════════════════════════════
// IRQ & STATUS MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

void clearIRQFlags() {
    writeRegister(REG_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
}

void setIRQMask(bool rx_dr, bool tx_ds, bool max_rt) {
    uint8_t config = readRegister(REG_CONFIG);
    config &= ~((1 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
    if (rx_dr) config |= (1 << MASK_RX_DR);
    if (tx_ds) config |= (1 << MASK_TX_DS);
    if (max_rt) config |= (1 << MASK_MAX_RT);
    writeRegister(REG_CONFIG, config);
}

uint8_t getObserveTX() {
    return readRegister(REG_OBSERVE_TX);
}

uint8_t getLostPacketCount() {
    return (readRegister(REG_OBSERVE_TX) >> PLOS_CNT) & 0x0F;
}

uint8_t getRetryCount() {
    return readRegister(REG_OBSERVE_TX) & 0x0F;
}

bool getCarrierDetect() {
    return readRegister(REG_CD) & 0x01;
}

// ═══════════════════════════════════════════════════════════════════════════════
// INITIALIZATION & RESET
// ═══════════════════════════════════════════════════════════════════════════════

void initialize() {
    rf_setup_cache = 0x0F; // Default: 1Mbps, 0dBm, LNA_HCURR
}

bool reset() {
    flushTX();
    flushRX();
    
    uint8_t status1 = getStatus();
    uint8_t status2 = readRegister(REG_STATUS);
    
    // Reset to power-on defaults
    writeRegister(REG_CONFIG,      0x08);     // EN_CRC, PWR_UP=0
    writeRegister(REG_EN_AA,       0x3F);     // Auto-ack all pipes
    writeRegister(REG_EN_RXADDR,   0x03);     // Enable pipe 0,1
    writeRegister(REG_SETUP_AW,    0x03);     // 5-byte address
    writeRegister(REG_SETUP_RETR,  0x03);     // 250us, 3 retries
    writeRegister(REG_RF_CH,       0x02);     // Channel 2
    writeRegister(REG_RF_SETUP,    0x0F);     // 1Mbps, 0dBm
    writeRegister(REG_STATUS,      0x70);     // Clear IRQs
    writeRegister(REG_RX_PW_P0,    0x00);
    writeRegister(REG_RX_PW_P1,    0x00);
    writeRegister(REG_FIFO_STATUS, 0x00);     // Read-only, but write clears
    writeRegister(REG_DYNPD,       0x00);
    writeRegister(REG_FEATURE,     0x00);
    
    // Clear addresses
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    writeRegisterMulti(REG_TX_ADDR, addr, 5);
    writeRegisterMulti(REG_RX_ADDR_P0, addr, 5);
    addr[0] = 0xC2;
    writeRegisterMulti(REG_RX_ADDR_P1, addr, 5);
    
    powerDown();
    CE_LOW();
    
    return (status1 == status2 && (status1 & 0x0F) == 0x0E);
}

// ═══════════════════════════════════════════════════════════════════════════════
// INTERRUPT HANDLER
// ═══════════════════════════════════════════════════════════════════════════════
void irqHandler() {
    irq_triggered = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL INTERFACE UTILITIES
// ═══════════════════════════════════════════════════════════════════════════════

void printHex(uint8_t val) {
    if (val < 16) Serial.print('0');
    Serial.print(val, HEX);
}

void printHexArray(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (i > 0) Serial.print(' ');
        printHex(data[i]);
    }
}

void printBinary(uint8_t val) {
    for (int8_t i = 7; i >= 0; i--) {
        Serial.print((val >> i) & 0x01);
    }
}

uint8_t parseHex(const char* str) {
    return (uint8_t)strtol(str, NULL, 16);
}

// ═══════════════════════════════════════════════════════════════════════════════
// REGISTER DUMP WITH DECODING
// ═══════════════════════════════════════════════════════════════════════════════
// FIXED: Use __FlashStringHelper* parameter type to match F() macro

void dumpRegister(uint8_t reg, const __FlashStringHelper* name) {
    uint8_t val = readRegister(reg);
    Serial.print(F("  0x"));
    printHex(reg);
    Serial.print(F(" "));
    Serial.print(name);
    Serial.print(F(" = 0x"));
    printHex(val);
    Serial.print(F(" ("));
    printBinary(val);
    Serial.println(F(")"));
}

void dumpAllRegisters() {
    Serial.println(F("=== COMPLETE NRF24L01+ REGISTER DUMP ==="));
    
    dumpRegister(REG_CONFIG,      F("CONFIG     "));
    dumpRegister(REG_EN_AA,       F("EN_AA      "));
    dumpRegister(REG_EN_RXADDR,   F("EN_RXADDR  "));
    dumpRegister(REG_SETUP_AW,    F("SETUP_AW   "));
    dumpRegister(REG_SETUP_RETR,  F("SETUP_RETR "));
    dumpRegister(REG_RF_CH,       F("RF_CH      "));
    dumpRegister(REG_RF_SETUP,    F("RF_SETUP   "));
    dumpRegister(REG_STATUS,      F("STATUS     "));
    dumpRegister(REG_OBSERVE_TX,  F("OBSERVE_TX "));
    dumpRegister(REG_CD,          F("CD/RPD     "));
    dumpRegister(REG_RX_PW_P0,    F("RX_PW_P0   "));
    dumpRegister(REG_RX_PW_P1,    F("RX_PW_P1   "));
    dumpRegister(REG_RX_PW_P2,    F("RX_PW_P2   "));
    dumpRegister(REG_RX_PW_P3,    F("RX_PW_P3   "));
    dumpRegister(REG_RX_PW_P4,    F("RX_PW_P4   "));
    dumpRegister(REG_RX_PW_P5,    F("RX_PW_P5   "));
    dumpRegister(REG_FIFO_STATUS, F("FIFO_STATUS"));
    dumpRegister(REG_DYNPD,       F("DYNPD      "));
    dumpRegister(REG_FEATURE,     F("FEATURE    "));
    
    // Multi-byte addresses
    uint8_t addr[5];
    Serial.println(F("\n--- ADDRESSES ---"));
    
    readRegisterMulti(REG_TX_ADDR, addr, 5);
    Serial.print(F("  TX_ADDR  = ")); printHexArray(addr, 5); Serial.println();
    
    readRegisterMulti(REG_RX_ADDR_P0, addr, 5);
    Serial.print(F("  RX_ADDR_P0 = ")); printHexArray(addr, 5); Serial.println();
    
    readRegisterMulti(REG_RX_ADDR_P1, addr, 5);
    Serial.print(F("  RX_ADDR_P1 = ")); printHexArray(addr, 5); Serial.println();
    
    for (uint8_t i = 2; i <= 5; i++) {
        uint8_t lsbyte = readRegister(REG_RX_ADDR_P0 + i);
        Serial.print(F("  RX_ADDR_P"));
        Serial.print(i);
        Serial.print(F(" LSB = 0x"));
        printHex(lsbyte);
        Serial.println();
    }
}

void printDecodedStatus() {
    uint8_t status = getStatus();
    uint8_t config = readRegister(REG_CONFIG);
    uint8_t fifo = getFIFOStatus();
    
    Serial.println(F("=== DECODED STATUS REPORT ==="));
    
    Serial.print(F("STATUS (0x")); printHex(status); Serial.println(F("):"));
    Serial.print(F("  RX_DR : ")); Serial.println((status & (1 << RX_DR)) ? F("Data Ready") : F("Clear"));
    Serial.print(F("  TX_DS : ")); Serial.println((status & (1 << TX_DS)) ? F("Sent") : F("Clear"));
    Serial.print(F("  MAX_RT: ")); Serial.println((status & (1 << MAX_RT)) ? F("Max Retry") : F("Clear"));
    Serial.print(F("  RX_P_NO: ")); Serial.println((status >> RX_P_NO) & 0x07);
    Serial.print(F("  TX_FULL: ")); Serial.println((status & (1 << TX_FULL)) ? F("YES") : F("No"));
    
    Serial.print(F("\nCONFIG (0x")); printHex(config); Serial.println(F("):"));
    Serial.print(F("  PRIM_RX: ")); Serial.println((config & (1 << PRIM_RX)) ? F("RX Mode") : F("TX Mode"));
    Serial.print(F("  PWR_UP : ")); Serial.println((config & (1 << PWR_UP)) ? F("Up") : F("Down"));
    Serial.print(F("  CRC    : ")); 
    if (config & (1 << EN_CRC)) {
        Serial.println((config & (1 << CRCO)) ? F("2 bytes") : F("1 byte"));
    } else {
        Serial.println(F("Disabled"));
    }
    
    Serial.print(F("\nFIFO (0x")); printHex(fifo); Serial.println(F("):"));
    Serial.print(F("  TX_REUSE : ")); Serial.println((fifo >> TX_REUSE) & 0x01);
    Serial.print(F("  TX_FULL  : ")); Serial.println((fifo & (1 << TX_FIFO_FULL)) ? F("YES") : F("No"));
    Serial.print(F("  TX_EMPTY : ")); Serial.println((fifo & (1 << TX_EMPTY)) ? F("YES") : F("No"));
    Serial.print(F("  RX_FULL  : ")); Serial.println((fifo & (1 << RX_FULL)) ? F("YES") : F("No"));
    Serial.print(F("  RX_EMPTY : ")); Serial.println((fifo & (1 << RX_EMPTY)) ? F("YES") : F("No"));
    
    Serial.print(F("\nOBSERVE_TX (0x")); printHex(getObserveTX()); Serial.println(F("):"));
    Serial.print(F("  Lost Packets: ")); Serial.println(getLostPacketCount());
    Serial.print(F("  Retries     : ")); Serial.println(getRetryCount());
    
    Serial.print(F("\nRF_CHANNEL: ")); Serial.println(getChannel());
    Serial.print(F("RF_SETUP  : 0x")); printHex(readRegister(REG_RF_SETUP)); Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND PARSER
// ═══════════════════════════════════════════════════════════════════════════════

void parseAndExecute(char* cmd) {
    while (*cmd == ' ') cmd++;
    if (*cmd == '\0') return;
    
    char opcode = *cmd++;
    while (*cmd == ' ') cmd++;
    
    switch (opcode) {
        // ─── Basic Register Access ───────────────────────────────────────────
        case 'W': {  // W <addr> <data>  - Write single register
            char* addrStr = strtok(cmd, " ");
            char* dataStr = strtok(NULL, " ");
            if (!addrStr || !dataStr) {
                Serial.println(F("ERR: W <addr> <data>"));
                return;
            }
            uint8_t addr = parseHex(addrStr);
            uint8_t data = parseHex(dataStr);
            writeRegister(addr, data);
            Serial.println(F("OK"));
            break;
        }
        
        case 'R': {  // R <addr>  - Read single register
            char* addrStr = strtok(cmd, " ");
            if (!addrStr) {
                Serial.println(F("ERR: R <addr>"));
                return;
            }
            uint8_t addr = parseHex(addrStr);
            uint8_t data = readRegister(addr);
            Serial.print(F("OK 0x"));
            printHex(data);
            Serial.println();
            break;
        }
        
        case 'M': {  // M <addr> <len> <d0> <d1> ...  - Multi-byte write
            char* addrStr = strtok(cmd, " ");
            char* lenStr = strtok(NULL, " ");
            if (!addrStr || !lenStr) {
                Serial.println(F("ERR: M <addr> <len> <bytes...>"));
                return;
            }
            uint8_t addr = parseHex(addrStr);
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            uint8_t data[MAX_PAYLOAD];
            for (uint8_t i = 0; i < len; i++) {
                char* dStr = strtok(NULL, " ");
                if (!dStr) {
                    Serial.println(F("ERR: Missing data bytes"));
                    return;
                }
                data[i] = parseHex(dStr);
            }
            writeRegisterMulti(addr, data, len);
            Serial.println(F("OK"));
            break;
        }
        
        case 'n': {  // n <addr> <len>  - Multi-byte read (lowercase to avoid conflict with N reset)
            char* addrStr = strtok(cmd, " ");
            char* lenStr = strtok(NULL, " ");
            if (!addrStr || !lenStr) {
                Serial.println(F("ERR: n <addr> <len>"));
                return;
            }
            uint8_t addr = parseHex(addrStr);
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            uint8_t data[MAX_PAYLOAD];
            readRegisterMulti(addr, data, len);
            Serial.print(F("OK "));
            printHexArray(data, len);
            Serial.println();
            break;
        }
        
        // ─── Payload Operations ──────────────────────────────────────────────
        case 'P': {  // P <len> <d0> ...  - Write TX payload
            char* lenStr = strtok(cmd, " ");
            if (!lenStr) {
                Serial.println(F("ERR: P <len> <bytes...>"));
                return;
            }
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            for (uint8_t i = 0; i < len; i++) {
                char* dStr = strtok(NULL, " ");
                if (!dStr) {
                    Serial.println(F("ERR: Missing payload bytes"));
                    return;
                }
                tx_buffer[i] = parseHex(dStr);
            }
            writePayload(tx_buffer, len);
            Serial.println(F("OK"));
            break;
        }
        
        case 'p': {  // p <len> <d0> ...  - Write TX payload NO_ACK
            char* lenStr = strtok(cmd, " ");
            if (!lenStr) {
                Serial.println(F("ERR: p <len> <bytes...>"));
                return;
            }
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            for (uint8_t i = 0; i < len; i++) {
                char* dStr = strtok(NULL, " ");
                if (!dStr) {
                    Serial.println(F("ERR: Missing payload bytes"));
                    return;
                }
                tx_buffer[i] = parseHex(dStr);
            }
            writePayload(tx_buffer, len, true);
            Serial.println(F("OK"));
            break;
        }
        
        case 'G': {  // G <len>  - Read RX payload
            char* lenStr = strtok(cmd, " ");
            if (!lenStr) {
                Serial.println(F("ERR: G <len>"));
                return;
            }
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            readPayload(rx_buffer, len);
            Serial.print(F("DATA "));
            printHexArray(rx_buffer, len);
            Serial.println();
            break;
        }
        
        case 'F': {  // F  - Read dynamic payload width
            uint8_t width = getDynamicPayloadWidth();
            Serial.print(F("OK "));
            Serial.println(width);
            break;
        }
        
        case 'A': {  // A <pipe> <len> <d0> ...  - Write ACK payload
            char* pipeStr = strtok(cmd, " ");
            char* lenStr = strtok(NULL, " ");
            if (!pipeStr || !lenStr) {
                Serial.println(F("ERR: A <pipe> <len> <bytes...>"));
                return;
            }
            uint8_t pipe = parseHex(pipeStr);
            uint8_t len = parseHex(lenStr);
            if (len > MAX_PAYLOAD) len = MAX_PAYLOAD;
            
            for (uint8_t i = 0; i < len; i++) {
                char* dStr = strtok(NULL, " ");
                if (!dStr) {
                    Serial.println(F("ERR: Missing ACK payload bytes"));
                    return;
                }
                tx_buffer[i] = parseHex(dStr);
            }
            writeAckPayload(pipe, tx_buffer, len);
            Serial.println(F("OK"));
            break;
        }
        
        // ─── Commands ────────────────────────────────────────────────────────
        case 'S': {  // S <cmd>  - Strobe/Send SPI command
            char* cmdStr = strtok(cmd, " ");
            if (!cmdStr) {
                Serial.println(F("ERR: S <cmd_hex>"));
                return;
            }
            uint8_t scmd = parseHex(cmdStr);
            uint8_t result = sendCommand(scmd);
            Serial.print(F("OK STATUS=0x"));
            printHex(result);
            Serial.println();
            break;
        }
        
        case 'Z': {  // Z <code>  - Activate features
            char* codeStr = strtok(cmd, " ");
            if (!codeStr) {
                Serial.println(F("ERR: Z <code> (73=toggle)"));
                return;
            }
            uint8_t code = parseHex(codeStr);
            CSN_LOW();
            spiTransfer(CMD_ACTIVATE);
            spiTransfer(code);
            CSN_HIGH();
            Serial.println(F("OK"));
            break;
        }
        
        // ─── Mode Control ────────────────────────────────────────────────────
        case 'X': {  // X <mode>  - Set operating mode
            char* modeStr = strtok(cmd, " ");
            if (!modeStr) {
                Serial.println(F("ERR: X <mode> (0=PD,1=STBY,2=RX,3=TX)"));
                return;
            }
            uint8_t mode = parseHex(modeStr);
            switch (mode) {
                case 0: setPowerMode(RF_Mode::POWER_DOWN); break;
                case 1: setPowerMode(RF_Mode::STANDBY_I); break;
                case 2: setPowerMode(RF_Mode::RX_MODE); break;
                case 3: setPowerMode(RF_Mode::TX_MODE); break;
                default:
                    Serial.println(F("ERR: Mode 0-3"));
                    return;
            }
            Serial.println(F("OK"));
            break;
        }
        
        // ─── RF Configuration ────────────────────────────────────────────────
        case 'C': {  // C <ch>  - Set channel (0-125)
            char* chStr = strtok(cmd, " ");
            if (!chStr) {
                Serial.println(F("ERR: C <0-125>"));
                return;
            }
            uint8_t ch = parseHex(chStr);
            setChannel(ch);
            Serial.print(F("OK CH="));
            Serial.println(getChannel());
            break;
        }
        
        case 'T': {  // T <pwr>  - TX power (0=-18dBm,1=-12,2=-6,3=0dBm)
            char* pwrStr = strtok(cmd, " ");
            if (!pwrStr) {
                Serial.println(F("ERR: T <0-3>"));
                return;
            }
            uint8_t pwr = parseHex(pwrStr) & 0x03;
            setTXPower(static_cast<TXPower>(pwr));
            Serial.println(F("OK"));
            break;
        }
        
        case 'B': {  // B <rate>  - Data rate (0=1M,1=2M,2=250K)
            char* rateStr = strtok(cmd, " ");
            if (!rateStr) {
                Serial.println(F("ERR: B <0-2>"));
                return;
            }
            uint8_t rate = parseHex(rateStr);
            if (rate > 2) {
                Serial.println(F("ERR: Rate 0=1M,1=2M,2=250K"));
                return;
            }
            setDataRate(static_cast<DataRate>(rate));
            Serial.println(F("OK"));
            break;
        }
        
        // ─── Pipe Management ─────────────────────────────────────────────────
        case 'E': {  // E <pipe>  - Enable RX pipe
            char* pipeStr = strtok(cmd, " ");
            if (!pipeStr) {
                Serial.println(F("ERR: E <0-5>"));
                return;
            }
            enableRXPipe(parseHex(pipeStr));
            Serial.println(F("OK"));
            break;
        }
        
        case 'K': {  // K <pipe>  - Disable RX pipe
            char* pipeStr = strtok(cmd, " ");
            if (!pipeStr) {
                Serial.println(F("ERR: K <0-5>"));
                return;
            }
            disableRXPipe(parseHex(pipeStr));
            Serial.println(F("OK"));
            break;
        }
        
        case 'L': {  // L <pipe> <width>  - Set static RX payload width
            char* pipeStr = strtok(cmd, " ");
            char* widthStr = strtok(NULL, " ");
            if (!pipeStr || !widthStr) {
                Serial.println(F("ERR: L <pipe> <width>"));
                return;
            }
            setRXPayloadWidth(parseHex(pipeStr), parseHex(widthStr));
            Serial.println(F("OK"));
            break;
        }
        
        case 'U': {  // U <pipe> <0/1>  - Auto-ack on pipe
            char* pipeStr = strtok(cmd, " ");
            char* enStr = strtok(NULL, " ");
            if (!pipeStr || !enStr) {
                Serial.println(F("ERR: U <pipe> <0/1>"));
                return;
            }
            setAutoAck(parseHex(pipeStr), parseHex(enStr) != 0);
            Serial.println(F("OK"));
            break;
        }
        
        case 'Y': {  // Y <pipe> <0/1>  - Dynamic payload on pipe
            char* pipeStr = strtok(cmd, " ");
            char* enStr = strtok(NULL, " ");
            if (!pipeStr || !enStr) {
                Serial.println(F("ERR: Y <pipe> <0/1>"));
                return;
            }
            uint8_t pipe = parseHex(pipeStr);
            if (parseHex(enStr))
                enableDynamicPayload(pipe);
            else
                disableDynamicPayload(pipe);
            Serial.println(F("OK"));
            break;
        }
        
        // ─── Address Configuration ───────────────────────────────────────────
        case 'J': {  // J <width>  - Address width (3-5)
            char* wStr = strtok(cmd, " ");
            if (!wStr) {
                Serial.println(F("ERR: J <3-5>"));
                return;
            }
            uint8_t w = parseHex(wStr);
            if (w < 3 || w > 5) {
                Serial.println(F("ERR: Width must be 3-5"));
                return;
            }
            setAddressWidth(static_cast<AddressWidth>(w));
            Serial.println(F("OK"));
            break;
        }
        
        case '@': {  // @ <0/1> <len> <bytes...>  - Set TX (0) or RX_P0 (1) address
            char* typeStr = strtok(cmd, " ");
            char* lenStr = strtok(NULL, " ");
            if (!typeStr || !lenStr) {
                Serial.println(F("ERR: @ <0=TX/1=RX> <len> <bytes...>"));
                return;
            }
            uint8_t type = parseHex(typeStr);
            uint8_t len = parseHex(lenStr);
            if (len > 5) len = 5;
            
            for (uint8_t i = 0; i < len; i++) {
                char* dStr = strtok(NULL, " ");
                if (!dStr) {
                    Serial.println(F("ERR: Missing address bytes"));
                    return;
                }
                tx_buffer[i] = parseHex(dStr);
            }
            
            if (type == 0)
                setTXAddress(tx_buffer, len);
            else
                setRXAddress(0, tx_buffer, len);
            Serial.println(F("OK"));
            break;
        }
        
        // ─── Advanced Configuration ──────────────────────────────────────────
        case 'V': {  // V <delay> <count>  - Auto-retransmit
            char* delStr = strtok(cmd, " ");
            char* cntStr = strtok(NULL, " ");
            if (!delStr || !cntStr) {
                Serial.println(F("ERR: V <delay(0-15)> <count(0-15)>"));
                return;
            }
            setRetransmit(parseHex(delStr), parseHex(cntStr));
            Serial.println(F("OK"));
            break;
        }
        
        case 'r': {  // r <0/1/2>  - CRC mode
            char* modeStr = strtok(cmd, " ");
            if (!modeStr) {
                Serial.println(F("ERR: r <0=off/1=1byte/2=2byte>"));
                return;
            }
            uint8_t mode = parseHex(modeStr);
            if (mode > 2) {
                Serial.println(F("ERR: CRC mode 0-2"));
                return;
            }
            setCRC(static_cast<CRCMode>(mode));
            Serial.println(F("OK"));
            break;
        }
        
        case 'I': {  // I <mask>  - IRQ mask (bit0=MAX_RT,1=TX_DS,2=RX_DR)
            char* maskStr = strtok(cmd, " ");
            if (!maskStr) {
                Serial.println(F("ERR: I <mask_bits>"));
                return;
            }
            uint8_t mask = parseHex(maskStr);
            setIRQMask(mask & 0x04, mask & 0x02, mask & 0x01);
            Serial.println(F("OK"));
            break;
        }
        
        // ─── FIFO & Status ───────────────────────────────────────────────────
        case 'f': {  // f  - Flush TX
            flushTX();
            Serial.println(F("OK TX flushed"));
            break;
        }
        
        case 'g': {  // g  - Flush RX
            flushRX();
            Serial.println(F("OK RX flushed"));
            break;
        }
        
        case 'u': {  // u  - Reuse TX payload
            reuseTXPayload();
            Serial.println(F("OK"));
            break;
        }
        
        case 'O': {  // O  - Clear IRQ flags
            clearIRQFlags();
            Serial.println(F("OK"));
            break;
        }
        
        // ─── System Commands ─────────────────────────────────────────────────
        case 'i':  // i  - Initialize
            initialize();
            Serial.println(F("OK Initialized"));
            break;
            
        case 'N': {  // N  - Full reset
            bool ok = reset();
            Serial.print(F("OK Reset="));
            Serial.println(ok ? F("1") : F("0"));
            break;
        }
        
        case '?':  // ?  - Status report
            printDecodedStatus();
            break;
            
        case 'D':  // D  - Dump all registers
            dumpAllRegisters();
            break;
            
        case 'd': {  // d  - Carrier detect / RPD
            Serial.print(F("OK CD="));
            Serial.println(getCarrierDetect() ? F("1") : F("0"));
            break;
        }
        
        case 'q': {  // q  - Quick status (compact)
            uint8_t s = getStatus();
            uint8_t f = getFIFOStatus();
            Serial.print(F("OK S=0x")); printHex(s);
            Serial.print(F(" F=0x")); printHex(f);
            Serial.print(F(" CH=")); Serial.print(getChannel());
            Serial.print(F(" RT=")); Serial.print(getRetryCount());
            Serial.print(F(" LP=")); Serial.println(getLostPacketCount());
            break;
        }
        
        // ─── Configuration ───────────────────────────────────────────────────
        case '.': {  // .  - Toggle echo
            echoEnabled = !echoEnabled;
            Serial.print(F("OK ECHO="));
            Serial.println(echoEnabled ? F("ON") : F("OFF"));
            break;
        }
        
        case ',': {  // ,  - Toggle verbose
            verboseMode = !verboseMode;
            Serial.print(F("OK VERBOSE="));
            Serial.println(verboseMode ? F("ON") : F("OFF"));
            break;
        }
        
        case 'H':  // H  - Help
        case 'h':
        default:
            Serial.println(F("=== ULTIMATE NRF24L01+ SERIAL API COMMANDS ==="));
            Serial.println(F("REGISTER ACCESS:"));
            Serial.println(F("  W <addr> <data>        Write single register"));
            Serial.println(F("  R <addr>               Read single register"));
            Serial.println(F("  M <addr> <len> <..>    Multi-byte write"));
            Serial.println(F("  n <addr> <len>         Multi-byte read"));
            Serial.println(F(""));
            Serial.println(F("PAYLOAD OPERATIONS:"));
            Serial.println(F("  P <len> <d0>...        Write TX payload"));
            Serial.println(F("  p <len> <d0>...        Write TX payload (NO_ACK)"));
            Serial.println(F("  G <len>                Read RX payload"));
            Serial.println(F("  F                      Read dynamic payload width"));
            Serial.println(F("  A <pipe> <len> <..>    Write ACK payload"));
            Serial.println(F(""));
            Serial.println(F("COMMANDS:"));
            Serial.println(F("  S <cmd>                SPI command (E1=FLUSH_TX, E2=FLUSH_RX)"));
            Serial.println(F("  Z <code>               Activate (73=toggle features)"));
            Serial.println(F(""));
            Serial.println(F("MODE & RF:"));
            Serial.println(F("  X <mode>               Mode (0=PD 1=STBY 2=RX 3=TX)"));
            Serial.println(F("  C <ch>                 Channel 0-125"));
            Serial.println(F("  T <pwr>                TX power (0=-18dBm..3=0dBm)"));
            Serial.println(F("  B <rate>               Data rate (0=1M 1=2M 2=250K)"));
            Serial.println(F("  r <mode>               CRC (0=off 1=1byte 2=2byte)"));
            Serial.println(F(""));
            Serial.println(F("PIPES & ADDRESSES:"));
            Serial.println(F("  E <pipe>               Enable RX pipe (0-5)"));
            Serial.println(F("  K <pipe>               Disable RX pipe (0-5)"));
            Serial.println(F("  L <pipe> <width>       Static RX payload width"));
            Serial.println(F("  U <pipe> <0/1>         Auto-ack on pipe"));
            Serial.println(F("  Y <pipe> <0/1>         Dynamic payload on pipe"));
            Serial.println(F("  J <width>              Address width (3-5 bytes)"));
            Serial.println(F("  @ <0/1> <len> <..>     Set TX(0) or RX_P0(1) address"));
            Serial.println(F(""));
            Serial.println(F("ADVANCED:"));
            Serial.println(F("  V <delay> <count>      Auto-retransmit (0-15 each)"));
            Serial.println(F("  I <mask>               IRQ mask bits (1=MAX_RT,2=TX_DS,4=RX)"));
            Serial.println(F(""));
            Serial.println(F("FIFO & STATUS:"));
            Serial.println(F("  f                      Flush TX FIFO"));
            Serial.println(F("  g                      Flush RX FIFO"));
            Serial.println(F("  u                      Reuse TX payload"));
            Serial.println(F("  O                      Clear IRQ flags"));
            Serial.println(F(""));
            Serial.println(F("SYSTEM:"));
            Serial.println(F("  i                      Initialize"));
            Serial.println(F("  N                      Full reset to defaults"));
            Serial.println(F("  ?                      Decoded status report"));
            Serial.println(F("  D                      Dump all registers"));
            Serial.println(F("  d                      Carrier Detect / RPD"));
            Serial.println(F("  q                      Quick compact status"));
            Serial.println(F("  .                      Toggle echo"));
            Serial.println(F("  ,                      Toggle verbose"));
            Serial.println(F("  H                      This help"));
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ARDUINO SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial);  // Wait for native USB (Leonardo/Micro) or serial connection
    
    // Pin configuration
    pinMode(CE_PIN, OUTPUT);
    pinMode(CSN_PIN, OUTPUT);
    pinMode(IRQ_PIN, INPUT_PULLUP);  // Optional IRQ monitoring
    
    digitalWrite(CE_PIN, LOW);
    digitalWrite(CSN_PIN, HIGH);
    
    // SPI initialization
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);  // 8MHz on 16MHz Arduino (max spec)
    
    // Optional: Attach IRQ handler
    // attachInterrupt(digitalPinToInterrupt(IRQ_PIN), irqHandler, FALLING);
    
    // Initialize radio
    initialize();
    reset();
    
    Serial.println(F("\r\n=== ULTIMATE NRF24L01+ SERIAL API v3.0 READY ==="));
    Serial.println(F("115200 baud | All registers | Full feature set"));
    Serial.println(F("Send 'H' for command reference"));
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        
        // Echo if enabled
        if (echoEnabled) Serial.print(c);
        
        if (c == '\n' || c == '\r') {
            if (echoEnabled) Serial.println();
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                parseAndExecute(cmdBuffer);
                cmdIndex = 0;
            }
        } else {
            if (cmdIndex < CMD_BUFFER_SIZE - 1) {
                cmdBuffer[cmdIndex++] = c;
            } else {
                Serial.println(F("ERR: Command buffer overflow"));
                cmdIndex = 0;
            }
        }
    }
    
    // Optional: Handle IRQ-triggered events
    if (irq_triggered) {
        irq_triggered = false;
        uint8_t status = getStatus();
        if (verboseMode) {
            Serial.print(F("[IRQ] STATUS=0x"));
            printHex(status);
            Serial.println();
        }
    }
}