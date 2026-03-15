//========================================================================================
//
// 
//
//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library internals include file
// Copyright (C) 2020 - 2026 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//========================================================================================
#include "LcsLib.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h> 
#include <EEPROM.h>

//========================================================================================
//
//
//
//========================================================================================
// The board is the only one on a bus for programming.
// Need a way to set the I2C address.
// have a command that will put the board in PROG mode.
// have a command to set the I2C address.
// RESET via watchdog ?
// have a command that puts the board back to OP mode.
// remember: add ".development" to the lib directory, so that it can be edited.


//=======================================================================================
// CRC for serial number validation...
//
//
//=======================================================================================
uint8_t crc8( const uint8_t *data, size_t len ) {
  
    uint8_t crc = 0x00;

    for (size_t i = 0; i < len; i++) {
      
        crc ^= data[ i ];

        for ( int b = 0; b < 8; b++ ) {
            if ( crc & 0x80 ) crc = (crc << 1) ^ 0x07;
            else              crc <<= 1;
        }
    }

    return crc;
}

//=======================================================================================
// Chip UID setup.
//
//
//=======================================================================================
uint64_t fnv1a64( uint8_t *data, uint8_t len ) {
  
    uint64_t hash = 0xcbf29ce484222325ULL;

    for( uint8_t i=0;i<len;i++ ) {
      
        hash ^= data[i];
        hash *= 0x100000001b3ULL;
    }

    return hash;
}

uint64_t buildLcsHwUID( ) {

    uint8_t uidBuf[ 10 ];
  
    uidBuf[0]   = SIGROW.SERNUM0;
    uidBuf[1]   = SIGROW.SERNUM1;
    uidBuf[2]   = SIGROW.SERNUM2;
    uidBuf[3]   = SIGROW.SERNUM3;
    uidBuf[4]   = SIGROW.SERNUM4;
    uidBuf[5]   = SIGROW.SERNUM5;
    uidBuf[6]   = SIGROW.SERNUM6;
    uidBuf[7]   = SIGROW.SERNUM7;
    uidBuf[8]   = SIGROW.SERNUM8;
    uidBuf[9]   = SIGROW.SERNUM9;

   return( fnv1a64( uidBuf, sizeof( uidBuf )));
}


//=======================================================================================
// Watchdog support. 
//
// WDT_PERIOD_8KCLK_gc → ca. 8 s
// WDT_PERIOD_4KCLK_gc → ca. 4 s
// WDT_PERIOD_2KCLK_gc → ca. 2 s
// WDT_PERIOD_1KCLK_gc → ca. 1 s
//
//=======================================================================================
void setupWatchdog( ) {
  
    wdt_enable( WDT_PERIOD_8KCLK_gc ); 
    wdt_reset( );
}

void feedWatchdog( ) {
  
    wdt_reset( );
}

bool wasWatchdogReset() {
    
    uint8_t flags = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = flags; 
    return ( flags & RSTCTRL_WDRF_bm );
}

//=======================================================================================
// EEPROM access
//
//=======================================================================================
void formatEEPROM( ) {

  // ??? just EEPROM.put fields...
  // ??? at the end update MAGIC and VERSION...
}

void loadFromEEPROM( ) {

  // ??? read the magic field.
  // ??? if OK read the complete header
  // ??? else formtEEPROM with default values.
}

uint16_t readField( uint8_t index ) {
  
    uint16_t value;
    int addr = index * sizeof(uint16_t);
    EEPROM.get(addr, value);
    return value;
}

void updateField( uint8_t index, uint16_t value ) {
  
    int addr = index * sizeof( uint16_t );
    EEPROM.put( addr, value );
}


//=======================================================================================
//
//
//
//=======================================================================================
// ??? habe libary functions for the device firmware writer ?
// ??? what is a good mental model ?











//=======================================================================================
//
//
//
//=======================================================================================
// 4x4 matrix scan

#if 0 

uint16_t raw_state;        // instantaneous scan
uint16_t stable_state;     // debounced result
uint8_t  debounce[16];     // per-key counters

// rows Port C

#define ROW_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

static void rows_init(void) {
    PORTC.DIRSET = ROW_MASK;
    PORTC.OUTSET = ROW_MASK;   // all inactive (HIGH)
}

// columns port A

#define COL_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

static void cols_init(void) {
    PORTA.DIRCLR = COL_MASK;

    // Enable pull-ups
    PORTA.PIN0CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
}

static inline void select_row(uint8_t row) {
    PORTC.OUTSET = ROW_MASK;
    PORTC.OUTCLR = (PIN0_bm << row);
}

static inline void select_row(uint8_t row) {
    PORTC.OUTSET = ROW_MASK;
    PORTC.OUTCLR = (PIN0_bm << row);
}

static uint16_t matrix_scan_raw(void) {
    uint16_t result = 0;

    for (uint8_t row = 0; row < 4; row++) {
        select_row(row);
        _delay_us(10);   // settle time

        uint8_t cols = (~PORTA.IN) & COL_MASK; // active LOW → invert

        for (uint8_t col = 0; col < 4; col++) {
            if (cols & (PIN0_bm << col)) {
                result |= (1 << (row * 4 + col));
            }
        }
    }

    return result;
}

#define DEBOUNCE_TICKS 5   // 5 × 10 ms = 50 ms

static void debounce_update(uint16_t raw) {
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t bit = (raw >> i) & 1;
        uint8_t stable = (stable_state >> i) & 1;

        if (bit == stable) {
            debounce[i] = 0;   // no change
        } else {
            if (++debounce[i] >= DEBOUNCE_TICKS) {
                stable_state ^= (1 << i);  // accept change
                debounce[i] = 0;
            }
        }
    }
}

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    rows_init();
    cols_init();

    stable_state = 0;

    while (1) {
        raw_state = matrix_scan_raw();
        debounce_update(raw_state);

        _delay_ms(10);
    }
}

#endif


//=======================================================================================
//
//
//
//=======================================================================================
// PWM servo stuff - version 1

#if 0

// include file...

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_MAX 8

void servo_init(void);

/* index: 0..SERVO_MAX-1
 * pin: bit number on PORTA (0..7)
 */
void servo_attach(uint8_t index, uint8_t pin);

/* pulse width in microseconds (500–2500 typical) */
void servo_write_us(uint8_t index, uint16_t us);

#endif


#include "servo.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define SERVO_FRAME_US 20000
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
#define SERVO_GAP_US   4     // tiny inter-servo gap

static volatile uint16_t pulse[SERVO_MAX];
static volatile uint8_t  mask[SERVO_MAX];
static volatile uint8_t  count;

static volatile uint8_t  idx;
static volatile uint8_t  phase;
static volatile uint16_t acc_time;

void servo_init(void)
{
    // GPIO (PORTA only, by design)
    PORTA.DIR |= 0xFF;

    // Default pulses
    for (uint8_t i = 0; i < SERVO_MAX; i++)
        pulse[i] = 1500;

    count = 0;
    idx = 0;
    phase = 0;
    acc_time = 0;

    // TCA0: normal mode, 1 µs tick
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
    TCA0.SINGLE.CNT   = 0;
    TCA0.SINGLE.CMP0  = 1000;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;

    // F_CPU = 4 MHz → DIV4 = 1 MHz
    TCA0.SINGLE.CTRLA =
        TCA_SINGLE_CLKSEL_DIV4_gc |
        TCA_SINGLE_ENABLE_bm;
}

void servo_attach(uint8_t index, uint8_t pin)
{
    if (index >= SERVO_MAX || pin > 7)
        return;

    mask[index] = (1 << pin);
    PORTA.DIR  |= (1 << pin);

    if (index >= count)
        count = index + 1;
}

void servo_write_us(uint8_t index, uint16_t us)
{
    if (index >= count)
        return;

    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    pulse[index] = us;
}

ISR(TCA0_CMP0_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;

    if (!count)
        return;

    if (phase == 0)
    {
        // pulse ON
        PORTA.OUT |= mask[idx];
        TCA0.SINGLE.CMP0 += pulse[idx];
        acc_time += pulse[idx];
        phase = 1;
    }
    else
    {
        // pulse OFF
        PORTA.OUT &= ~mask[idx];
        idx++;

        if (idx >= count)
        {
            // frame gap
            uint16_t gap = SERVO_FRAME_US - acc_time;
            TCA0.SINGLE.CMP0 += gap;
            acc_time = 0;
            idx = 0;
        }
        else
        {
            TCA0.SINGLE.CMP0 += SERVO_GAP_US;
            acc_time += SERVO_GAP_US;
        }

        phase = 0;
    }
}

#endif


//=======================================================================================
//
//
//
//=======================================================================================
// PWM servo stuff - version 1

#if 0

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <avr/io.h>

#define SERVO_MAX 8

void servo_init(void);

/* index: 0..SERVO_MAX-1
 * port : &PORTA, &PORTB, ...
 * pin  : bit number (0..7)
 */
void servo_attach(uint8_t index, PORT_t *port, uint8_t pin);

void servo_write_us(uint8_t index, uint16_t us);

#endif



#include "servo.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define SERVO_FRAME_US 20000
#define SERVO_MIN_US   500
#define SERVO_MAX_US   2500
#define SERVO_GAP_US   4


static volatile uint16_t pulse[SERVO_MAX];
static volatile uint8_t  mask[SERVO_MAX];
static PORT_t           *port[SERVO_MAX];

static volatile uint8_t  count;
static volatile uint8_t  idx;
static volatile uint8_t  phase;
static volatile uint16_t acc_time;



void servo_init(void)
{
    for (uint8_t i = 0; i < SERVO_MAX; i++)
        pulse[i] = 1500;

    count = 0;
    idx = 0;
    phase = 0;
    acc_time = 0;

    // TCA0: 1 µs tick (F_CPU = 4 MHz)
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
    TCA0.SINGLE.CNT   = 0;
    TCA0.SINGLE.CMP0  = 1000;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;

    TCA0.SINGLE.CTRLA =
        TCA_SINGLE_CLKSEL_DIV4_gc |
        TCA_SINGLE_ENABLE_bm;
}

void servo_attach(uint8_t index, PORT_t *p, uint8_t pin)
{
    if (index >= SERVO_MAX || pin > 7)
        return;

    port[index] = p;
    mask[index] = (1 << pin);

    p->DIR |= mask[index];

    if (index >= count)
        count = index + 1;
}

void servo_write_us(uint8_t index, uint16_t us)
{
    if (index >= count)
        return;

    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    pulse[index] = us;
}


ISR(TCA0_CMP0_vect)
{
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;

    if (!count)
        return;

    if (phase == 0)
    {
        port[idx]->OUTSET = mask[idx];
        TCA0.SINGLE.CMP0 += pulse[idx];
        acc_time += pulse[idx];
        phase = 1;
    }
    else
    {
        port[idx]->OUTCLR = mask[idx];
        idx++;

        if (idx >= count)
        {
            uint16_t gap = SERVO_FRAME_US - acc_time;
            TCA0.SINGLE.CMP0 += gap;
            acc_time = 0;
            idx = 0;
        }
        else
        {
            TCA0.SINGLE.CMP0 += SERVO_GAP_US;
            acc_time += SERVO_GAP_US;
        }

        phase = 0;
    }
}


#endif

//=======================================================================================
//
//
//
//=======================================================================================
// anotehr servo version 

#if 0 


0   Servo0.lower
2   Servo0.upper
4   Servo0.transitionTime
6   Servo1.lower
8   Servo1.upper
10  Servo1.transitionTime

struct ServoConfig {
    uint16_t lower;      // Untere Grenze
    uint16_t upper;      // Obere Grenze
    uint16_t transitionTime; // in ms
};

struct ServoState {
    uint16_t current;    // aktueller PWM Wert
    uint16_t target;     // Ziel PWM Wert
    uint32_t lastUpdate; // millis() der letzten Aktualisierung
};

ServoConfig servoConfig[2]; // RAM
ServoState servoState[2];   // RAM

#include <EEPROM.h>

void loadServoConfig() {
    for (uint8_t i=0; i<2; i++) {
        int addr = i*6;
        EEPROM.get(addr, servoConfig[i]);
        // sanity check
        if (servoConfig[i].lower >= servoConfig[i].upper) {
            servoConfig[i] = {1000, 2000, 1000}; // default
            EEPROM.put(addr, servoConfig[i]);
        }
        servoState[i].current = servoConfig[i].lower; // Default Position
        servoState[i].target = servoState[i].current;
        servoState[i].lastUpdate = millis();
    }
}



void saveServoConfig(uint8_t i) {
    int addr = i*6;
    EEPROM.put(addr, servoConfig[i]);
}

void setServoTarget(uint8_t i, uint16_t value) {
    if (value < servoConfig[i].lower) value = servoConfig[i].lower;
    if (value > servoConfig[i].upper) value = servoConfig[i].upper;
    servoState[i].target = value;
}

void updateServos() {
    uint32_t now = millis();
    static uint8_t lastMoved = 0;

    uint8_t i = (lastMoved == 0) ? 1 : 0; // alternierend
    ServoState &s = servoState[i];
    ServoConfig &c = servoConfig[i];

    if (s.current != s.target) {
        uint32_t dt = now - s.lastUpdate;
        if (dt == 0) return;

        int32_t diff = s.target - s.current;
        int32_t step = diff * dt / c.transitionTime;
        if (step == 0) step = (diff>0)?1:-1;

        s.current += step;
        if ((diff>0 && s.current > s.target) || (diff<0 && s.current < s.target))
            s.current = s.target;

        s.lastUpdate = now;

        // hier echten PWM setzen
        analogWrite(i, s.current);

        lastMoved = i;
    }
}

void setup() {
    pinMode(5, OUTPUT); // Servo0
    pinMode(6, OUTPUT); // Servo1
    loadServoConfig();
    analogWrite(5, servoState[0].current);
    analogWrite(6, servoState[1].current);
}

void loop() {
    updateServos();
    // Hier könnte I2C oder andere Logik laufen
}

#endif

//=======================================================================================
//
//
//
//=======================================================================================
// Servo improvement...
#if 0

struct Servo {
    uint16_t current;
    uint16_t target;

    uint16_t lower;
    uint16_t upper;

    uint16_t transition_ms;

    uint32_t start_time;
    uint16_t start_value;
};

Servo servo[2];

// start movement ...

void servo_set_target(uint8_t i, uint16_t value)
{
    if (value < servo[i].lower) value = servo[i].lower;
    if (value > servo[i].upper) value = servo[i].upper;

    servo[i].target = value;
    servo[i].start_value = servo[i].current;
    servo[i].start_time = millis();
}

// calculate position

void servo_update(uint8_t i)
{
    Servo &s = servo[i];

    if (s.current == s.target)
        return;

    uint32_t now = millis();
    uint32_t dt = now - s.start_time;

    if (dt >= s.transition_ms)
    {
        s.current = s.target;
    }
    else
    {
        int32_t diff = (int32_t)s.target - s.start_value;
        s.current = s.start_value + diff * dt / s.transition_ms;
    }

    analogWrite(i, s.current);
}

// schedule alternate

void servo_scheduler()
{
    static uint8_t next = 0;
    static uint32_t last_tick = 0;

    uint32_t now = millis();

    if (now - last_tick < 10)   // 10 ms Tick
        return;

    last_tick = now;

    servo_update(next);

    next ^= 1;  // 0 -> 1 -> 0 -> 1
}


void loop()
{
    servo_scheduler();

    handleI2C();

    Watchdog::feed();
}

#endif

//=======================================================================================
//
//
//
//=======================================================================================
// another one ... servos do not move simulatenously... smooth no blocking delays
// no interrupt handler needed... watch dog protects from hangs...

#if 0 

#include <EEPROM.h>

struct ServoConfig {
    uint16_t lower;
    uint16_t upper;
    uint16_t time_ms;
};

struct ServoState {
    uint16_t current;
    uint16_t target;
    uint16_t start;
    uint32_t t0;
};

ServoConfig cfg[2];
ServoState st[2];

const ServoConfig defaultCfg[2] =
{
    {1000,2000,1000},
    {1000,2000,1000}
};


void servo_load_config()
{
    for (uint8_t i=0;i<2;i++)
    {
        EEPROM.get(i*sizeof(ServoConfig), cfg[i]);

        if (cfg[i].lower >= cfg[i].upper)
        {
            cfg[i] = defaultCfg[i];
            EEPROM.put(i*sizeof(ServoConfig), cfg[i]);
        }

        st[i].current = cfg[i].lower;
        st[i].target  = cfg[i].lower;
        st[i].start   = st[i].current;
        st[i].t0      = millis();
    }
}

void servo_set(uint8_t i,uint16_t value)
{
    if(value<cfg[i].lower) value=cfg[i].lower;
    if(value>cfg[i].upper) value=cfg[i].upper;

    st[i].target=value;
    st[i].start=st[i].current;
    st[i].t0=millis();
}

void servo_update(uint8_t i)
{
    auto &s=st[i];
    auto &c=cfg[i];

    if(s.current==s.target) return;

    uint32_t dt=millis()-s.t0;

    if(dt>=c.time_ms)
        s.current=s.target;
    else
    {
        int32_t diff=(int32_t)s.target-s.start;
        s.current=s.start+diff*dt/c.time_ms;
    }

    analogWrite(i,s.current);
}

void servo_scheduler()
{
    static uint8_t next=0;
    static uint32_t last=0;

    uint32_t now=millis();

    if(now-last<10) return;   // 10ms tick
    last=now;

    servo_update(next);

    next^=1;
}

void watchdog_begin()
{
    _PROTECTED_WRITE(WDT.CTRLA,WDT_PERIOD_64CLK_gc);
}

inline void watchdog_feed()
{
    __builtin_avr_wdr();
}

void setup()
{
    pinMode(0,OUTPUT);
    pinMode(1,OUTPUT);

    servo_load_config();

    analogWrite(0,st[0].current);
    analogWrite(1,st[1].current);

    watchdog_begin();
}

void loop()
{
    servo_scheduler();

    handleI2C();   // deine Steuerlogik

    watchdog_feed();
}

#endif


//=======================================================================================
//
//
//
//=======================================================================================
// use a I2C slave library, small footprint. Later.
// For now, stick to Arduino libraries.

// A simple lib ? 
#if 0 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define I2C_ADDR 0x30
#define LED_PIN  PIN3_bm

#define REG_LED     0
#define REG_STATUS  1
#define REG_CMD     2
#define REG_DATA    3

#define STATUS_TIMEOUT   (1 << 0)
#define STATUS_BAD_REG   (1 << 1)
#define STATUS_GENCALL   (1 << 2)

volatile uint8_t regs[4];
volatile uint8_t reg_index = 0;
volatile bool expecting_reg = true;

volatile uint8_t i2c_idle_ticks = 0;

/* ---------------- GPIO ---------------- */

static void gpio_init(void) {
    PORTA.DIRSET = LED_PIN;
    PORTA.OUTCLR = LED_PIN;
}

/* ---------------- RTC timeout ---------------- */

static void rtc_init(void) {
    // 16 ms periodic interrupt
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
    RTC.PITCTRLA = RTC_PERIOD_CYC16_gc | RTC_PITEN_bm;
    RTC.PITINTCTRL = RTC_PI_bm;
}

ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;

    if (++i2c_idle_ticks > 10) { // ~160 ms
        regs[REG_STATUS] |= STATUS_TIMEOUT;
    }
}

/* ---------------- I2C ---------------- */

static void i2c_init(void) {
    TWI0.SADDR = I2C_ADDR << 1;

    TWI0.SCTRLA =
        TWI_ENABLE_bm |
        TWI_APIEN_bm |
        TWI_DIEN_bm |
        TWI_PIEN_bm;   // STOP interrupt

    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
}

ISR(TWI0_TWIS_vect) {
    uint8_t status = TWI0.SSTATUS;

    // Reset timeout counter on any I2C activity
    i2c_idle_ticks = 0;

    /* Address or STOP */
    if (status & TWI_APIF_bm) {

        if (status & TWI_AP_bm) {
            expecting_reg = true;

            if (status & TWI_GENCALL_bm) {
                regs[REG_STATUS] |= STATUS_GENCALL;
            }
        }

        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
    }

    /* Data phase */
    if (status & TWI_DIF_bm) {

        // Master write
        if (!(status & TWI_DIR_bm)) {
            uint8_t data = TWI0.SDATA;

            if (expecting_reg) {
                if (data < 4) {
                    reg_index = data;
                } else {
                    regs[REG_STATUS] |= STATUS_BAD_REG;
                    reg_index = 0;
                }
                expecting_reg = false;
            } else {
                regs[reg_index] = data;
            }

            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        }
        // Master read (not allowed on General Call)
        else {
            if (status & TWI_GENCALL_bm) {
                TWI0.SDATA = 0xFF;
            } else {
                TWI0.SDATA = regs[reg_index];
            }

            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        }
    }
}

/* ---------------- Main ---------------- */

int main(void) {
    gpio_init();
    i2c_init();
    rtc_init();

    sei();

    while (1) {
        /* LED control */
        if (regs[REG_LED]) {
            PORTA.OUTSET = LED_PIN;
        } else {
            PORTA.OUTCLR = LED_PIN;
        }

        /* Command handling */
        if (regs[REG_CMD] & 0x01) {
            regs[REG_STATUS] = 0;
            regs[REG_CMD] = 0;
        }
    }
}
#endif


//=======================================================================================
//
//
//
//=======================================================================================
// I2C 

#if 0

// app layer and I2C layer share a common memory map...
// an array of 16 bit words

// Burst
// Auto-Increment
// RO Maske
// Trennung Applikation / Transport
// Erweiterbar

// master write: I2Cadr, start reg, data words...
// ??? pointer auto increment

// master read: i2CAdr, start reg
// master reads 2 x N Bytes
// ??? pointer auto increment



#include <Wire.h>

#define I2C_ADDRESS 0x10
#define NUM_REGS 32

volatile uint16_t registers[NUM_REGS];
volatile uint8_t reg_ptr = 0;

// Bitmaske für Read-Only (1 = readonly)
const uint32_t reg_ro_mask = 
    (1UL << 0) |   // reg0 readonly
    (1UL << 1);    // reg1 readonly


void receiveEvent(int howMany)
{
    if (howMany < 1) return;

    uint8_t start_reg = Wire.read();
    howMany--;

    if (start_reg >= NUM_REGS)
        return;

    reg_ptr = start_reg;

    // Nur Pointer-Set?
    if (howMany == 0)
        return;

    // Burst Write
    while (howMany >= 2 && reg_ptr < NUM_REGS)
    {
        uint8_t low = Wire.read();
        uint8_t high = Wire.read();
        howMany -= 2;

        if (!(reg_ro_mask & (1UL << reg_ptr)))
        {
            uint16_t value = (uint16_t)low | ((uint16_t)high << 8);
            registers[reg_ptr] = value;
        }

        reg_ptr++;
    }

    // Rest verwerfen
    while (Wire.available()) Wire.read();
}  

void requestEvent()
{
    if (reg_ptr >= NUM_REGS)
    {
        Wire.write((uint8_t)0);
        Wire.write((uint8_t)0);
        return;
    }

    uint16_t value = registers[reg_ptr];

    Wire.write((uint8_t)(value & 0xFF));
    Wire.write((uint8_t)(value >> 8));

    reg_ptr++;
}

uint16_t safe_read(uint8_t index)
{
    uint16_t value;
    noInterrupts();
    value = registers[index];
    interrupts();
    return value;
}


void setup()
{
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    // Initialwerte
    registers[0] = 0x1234;   // Status (RO)
    registers[1] = 0x0001;   // Version (RO)
}

void loop()
{
    // Beispiel:
    // reg2 = LED control
    if (registers[2] & 1)
        PORTB.OUTSET = PIN3_bm;
    else
        PORTB.OUTCLR = PIN3_bm;

    // reg0 = Status counter
    registers[0]++;

    delay(10);
}


// PICO example:

// write two registers..

uint8_t buffer[5];
buffer[0] = 2;           // start register
buffer[1] = 0x34;        // low
buffer[2] = 0x12;        // high
buffer[3] = 0x78;
buffer[4] = 0x56;

i2c_write_blocking(I2C_PORT, SLAVE_ADDR, buffer, 5, false);

// read two regs 

uint8_t reg = 2;
uint8_t rx[4];

i2c_write_blocking(I2C_PORT, SLAVE_ADDR, &reg, 1, true);
i2c_read_blocking(I2C_PORT, SLAVE_ADDR, rx, 4, false);

uint16_t r2 = rx[0] | (rx[1] << 8);
uint16_t r3 = rx[2] | (rx[3] << 8);

#endif
