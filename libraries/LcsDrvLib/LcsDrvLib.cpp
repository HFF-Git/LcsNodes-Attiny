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
#include "arduino.h"
#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h> 
#include <Wire.h>
#include <EEPROM.h>
#include "LcsDrvLib.h"
#include "LcsDrvDesc.h"

//----------------------------------------------------------------------------------------
// Local name space
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
  const uint16_t  DRV_MWORD           = 0xa5a5;

//----------------------------------------------------------------------------------------
// The LcsBoardHeader structure defines what the board actually represents. It is 
// also the first structure that can be found on the controller board EEPROM. An 
// Atmega Attiny controller board also has the nice property of a serial number. 
// The header structure is 16 bytes long and matches exactly what is used in the 
// PICO controller world.
//
// The options fields are board specific information. For the Attiny, we store 
// a couple of flags and the confugured I2C address.
// 
//----------------------------------------------------------------------------------------
struct LcsDrvHeader {

    uint16_t            boardMword;                 // 0  - magic word
    uint16_t            boardType;                  // 1  - family/type/subtype
    uint16_t            boardVersion;               // 2  - major / sub version
    uint16_t            serialNum1;                 // 3  - serial number part 1
    uint16_t            serialNum2;                 // 4  - serial number part 2
    uint16_t            serialNum3;                 // 5  - serial number part 3
    uint16_t            serialNum4;                 // 6  - serial number part 4  
    uint16_t            boardOptions;               // 7  - options, board specific
};

//----------------------------------------------------------------------------------------
// The attribute storage. It is used by the I2C layer to store and read data and by the
// access function for the upper firmware layer.
//
//----------------------------------------------------------------------------------------
volatile uint16_t drvAttributes[ MAX_DRV_ATTRIBUTES ];

//----------------------------------------------------------------------------------------
// The driver header structure. This structure is the first structure in the EEPROM.
// We read it in when we initialize the driver. 
//
//----------------------------------------------------------------------------------------
LcsDrvHeader drvHeader;

//----------------------------------------------------------------------------------------
// I2C data structures. We exchange data packets with 3 or 5 bytes. The master sends 
// a 3 byte data packet for writing attributes and a 5 byte packet for a request with
// two 16-bit arguments. This allows use to cleanly identify how to react to the write
// operation. A write of one byte starts a read request. The byte containts the command
// code which is used to decide of we return an attribute, a two byte answer, or the
// reqzest data, which is a five byte answer consisting of status byte and two 16-bit
// arguments.  
//
// WRITE-ATTR:  i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H
// READ-ATTR:   i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H 
// 
// REQ-SEND: i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H,i2cArg1-L,i2cArg1-H
// REQ-RECV: i2cAdr,i2cCommand,i2cStatus,i2cArg0-L,i2cArg0-H,i2cArg1-L,i2cArg1-H
//----------------------------------------------------------------------------------------
uint8_t           i2cAdr;
volatile uint8_t  i2cWriteCmd;
volatile uint8_t  i2cReadCmd;
volatile uint8_t  i2cStatus;
volatile uint16_t i2cArg0;
volatile uint16_t i2cArg1;

//----------------------------------------------------------------------------------------
// CRC for serial number validation...
//
//----------------------------------------------------------------------------------------
uint8_t crc8( const uint8_t *data, size_t len ) {
  
    uint8_t crc = 0x00;

    for ( size_t i = 0; i < len; i++ ) {
      
        crc ^= data[ i ];

        for ( int b = 0; b < 8; b++ ) {
          
            if ( crc & 0x80 ) crc = ( crc << 1 ) ^ 0x07;
            else              crc <<= 1;
        }
    }

    return crc;
}

//----------------------------------------------------------------------------------------
// Hash function for building a 64-bit value from teh buffer input.
//
//----------------------------------------------------------------------------------------
uint64_t fnv1a64( uint8_t *data, uint8_t len ) {
  
    uint64_t hash = 0xcbf29ce484222325ULL;

    for ( uint8_t i=0;i<len;i++ ) {
      
        hash ^= data[i];
        hash *= 0x100000001b3ULL;
    }

    return hash;
}

//----------------------------------------------------------------------------------------
// Chip UID setup. The attiny chip has a serial number. We construct a unique 64-bit 
// value from that data.
//
//----------------------------------------------------------------------------------------
uint64_t buildHwUID( ) {

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

//----------------------------------------------------------------------------------------
// Setup the whatchdog. The moment that function is called, the watchdog timer needs to
// be resetted periodically.  
//
// WDT_PERIOD_8KCLK_gc → ca. 8 s
// WDT_PERIOD_4KCLK_gc → ca. 4 s
// WDT_PERIOD_2KCLK_gc → ca. 2 s
// WDT_PERIOD_1KCLK_gc → ca. 1 s
//
//----------------------------------------------------------------------------------------
void setupWatchdog( ) {
  
    wdt_enable( WDT_PERIOD_8KCLK_gc ); 
    wdt_reset( );
}

//----------------------------------------------------------------------------------------
// "formatEEPROM" creates a default memory structure and stores it to the EEPROM. This 
// function is typically called when the EEPROM is either brand new or was corrupted
// somehow.
//
//----------------------------------------------------------------------------------------
uint8_t formatEEPROM( ) {

  LcsDrvHeader tmp;

  uint64_t hwUID = buildHwUID( );

  tmp.boardMword       = DRV_MWORD;                   
  tmp.boardType        = DRV_TYPE;                                  
  tmp.boardVersion     = DRV_VERSION;                 
  tmp.serialNum1       = hwUID & 0xFFFF;                    
  tmp.serialNum2       = ( hwUID >> 16 ) & 0xFFFF;                    
  tmp.serialNum3       = ( hwUID >> 32 ) & 0xFFFF;             
  tmp.serialNum4       = ( hwUID >> 48 ) & 0xFFFF;   
  tmp.boardOptions     = 0;

  for ( int i = 0; i < MAX_DRV_ATTRIBUTES; i++ ) drvAttributes[ i ] = 0;

  EEPROM.put( 0, tmp );
  EEPROM.put( sizeof( LcsDrvHeader ), drvAttributes );
   
  return( 0 );
}

//----------------------------------------------------------------------------------------
// After reset, load the memory data structure with the content from the EEPROM. We first
// read the header data and check if they mach the program defied constants. If valid, 
// the remaining data is loaded from the EEPROM. If not, we format the EEPROM with default
// content. In either case, we return with a valid data setup.
// 
//----------------------------------------------------------------------------------------
uint8_t loadFromEEPROM( ) {

    EEPROM.get( 0, drvHeader );
    if (( drvHeader.boardMword != DRV_MWORD ) ||
        ( drvHeader.boardType != DRV_TYPE ) ||
        ( drvHeader.boardVersion != DRV_VERSION )) {

        formatEEPROM( );
    }
  
  EEPROM.get( sizeof( LcsDrvHeader ), drvAttributes );  
  return ( 0 );
}

// ??? needed ?
//----------------------------------------------------------------------------------------
// Read a word from the EEPROM. We view the EEPROM as an array of 16-bit words.
//
//----------------------------------------------------------------------------------------
uint16_t readField( uint8_t index ) {
  
    uint16_t value;
    int addr = index * sizeof( uint16_t );
    EEPROM.get( addr, value) ;
    return value;
}

// ??? needed ?
//----------------------------------------------------------------------------------------
// Write a word to the EEPROM. We view the EEPROM as an array of 16-bit words.
//
//----------------------------------------------------------------------------------------
void updateField( uint8_t index, uint16_t value ) {
  
    int addr = index * sizeof( uint16_t );
    EEPROM.put( addr, value );
}

//----------------------------------------------------------------------------------------
// The I2C channel receiver callback. We are informed that there is data. By proocol 
// definition there are exactly three date sizes. A size of one represents just the 
// command byte. We look at the start of a read sequence. A data size of three represents
// the command byte and two data bytes. This is mapped to an attribute read. We need 
// however to check that the command code matches. Finally, a data size of five is a 
// driver request. We fill the arguent area and let the upper firmware layer handle it. 
//
//----------------------------------------------------------------------------------------
void receiveEvent( int numOfBytes ) {

    if ( numOfBytes == 0 ) return;
 
    if ( numOfBytes == 1 ) {

      i2cReadCmd  = Wire.read( );
      i2cWriteCmd = DRV_CMD_IDLE;
    }
    else if ( numOfBytes == 3 ) {

        i2cWriteCmd = Wire.read( );
        i2cReadCmd  = DRV_CMD_IDLE;
    
        if ( i2cWriteCmd <= DRV_CMD_ATTR_END ) {

            uint8_t l = Wire.read( );
            uint8_t h = Wire.read( );
            drvAttributes[ i2cWriteCmd ] = ( l | ( h << 8 ));
        
            i2cReadCmd  = DRV_CMD_IDLE;
            i2cWriteCmd = DRV_CMD_IDLE;
        }
    }
    else if ( numOfBytes == 5 ) {

        i2cWriteCmd = Wire.read( );
        i2cReadCmd  = DRV_CMD_IDLE;
      
        if (( i2cWriteCmd >= DRV_CMD_REQ_START ) && ( i2cWriteCmd <= DRV_CMD_REQ_END )) {
      
            uint16_t arg0 = Wire.read( ) | ( Wire.read( ) << 8 );
            uint16_t arg1 = Wire.read( ) | ( Wire.read( ) << 8 );

            i2cArg0   = arg0;
            i2cArg1   = arg1;
            i2cStatus = 0;
        }
    } 
    else {

      i2cWriteCmd = DRV_CMD_IDLE;
      i2cReadCmd  = DRV_CMD_IDLE;
      while ( Wire.available( )) Wire.read( );
    }
}

//----------------------------------------------------------------------------------------
// The I2C channel master requests data. Serve thy master bidding. We have already 
// received the command byte and now we need to return the requested data. For a command
// code for attribute fetch, we just return the attribute data. For a request command
// code, we return the status and the two arguments. 
//
// Note that we cannot wait for the upper layer to finish the reqtesed operation. When
// the master requests a read, we will return data. The key is that the status field
// will not signal completion. So, the master will typically loop reading the packet
// until it has a status code of success.
//
//----------------------------------------------------------------------------------------
void requestEvent( ) {

  if ( i2cReadCmd < DRV_CMD_ATTR_END ) {
    
        uint16_t val = drvAttributes[ i2cReadCmd ];
        Wire.write( lowByte( val ));
        Wire.write( highByte( val ));
    } 
    else {
      
        Wire.write( i2cStatus );
        Wire.write( lowByte( i2cArg0 ));
        Wire.write( highByte( i2cArg0 ));
        Wire.write( lowByte( i2cArg1 ));
        Wire.write( highByte( i2cArg1 ));
    }

    i2cWriteCmd = DRV_CMD_IDLE;
    i2cReadCmd  = DRV_CMD_IDLE;
}

//----------------------------------------------------------------------------------------
// Setup the I2C channel. We use the I2C address from the header structure and set up
// the callbacks.
//
//----------------------------------------------------------------------------------------
uint8_t initI2cChannel( ) {

  Wire.begin( i2cAdr );
  delay( 10 );
  
  i2cWriteCmd   = DRV_CMD_IDLE;
  i2cReadCmd    = DRV_CMD_IDLE;
  i2cStatus     = 0;
  i2cArg0       = 0;
  i2cArg1       = 0;
   
  Wire.onReceive( receiveEvent );
  Wire.onRequest( requestEvent );
  return( 0 );
}

} // namespace


//=======================================================================================
// Library functions, externally visible.
//
//
//=======================================================================================

//----------------------------------------------------------------------------------------
// Routine to "feed" the watchdog monster periodically.
//
//----------------------------------------------------------------------------------------
void feedWatchdog( ) {
  
    wdt_reset( );
}

//----------------------------------------------------------------------------------------
// On startup, we can check if the reset was originated from the watchdog facility.
//
//----------------------------------------------------------------------------------------
bool wasWatchdogReset( ) {
    
    uint8_t flags = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = flags; 
    return ( flags & RSTCTRL_WDRF_bm );
}

//----------------------------------------------------------------------------------------
// The upper firmware layer will periodically call this procedure to check for work. This
// is a request with two arguments. We run this interrupts disabled to avoid ugly race
// conditions with the ISR handlers for the I2C interface. A positive return value means
// that there is work to do.
//
//----------------------------------------------------------------------------------------
int i2cGetRequest( uint8_t *cmd, uint16_t *arg0, uint16_t *arg1 ) {

    if ( i2cWriteCmd != DRV_CMD_IDLE ) {
    
        cli( );
        *cmd  = i2cWriteCmd;
        *arg0 = i2cArg0;
        *arg1 = i2cArg1;
        sei( );

        return ( 1 );
    }
    else return( 0 ); 
}

//----------------------------------------------------------------------------------------
// The upper firmware layer call this procedure to provide the request return. We also 
// run interrupts disabled and set the reply fields for the I2C code to provide when the
// master ask for it.
//
//----------------------------------------------------------------------------------------
void i2cSetResponse( uint8_t rStat, uint16_t r0, uint16_t r1 ) {

    cli( );
    i2cStatus = rStat;
    i2cArg0   = r0;
    i2cArg1   = r1;
    sei( );
}

//----------------------------------------------------------------------------------------
// A routine to retrieve an attribute from the attrbute array. Note that we disable
// interrupts shortly, since the array is a volatile structure. If the index is not
// within range, a zero value is returned.
//
//----------------------------------------------------------------------------------------
uint16_t getAttr( uint8_t index ) {

    uint16_t tmp;

    if ( index < MAX_DRV_ATTRIBUTES ) {

        cli( ); 
        tmp = drvAttributes[ index ];
        sei( );

        return ( tmp );
    }
    else return( 0 );
}

//----------------------------------------------------------------------------------------
// A routine to update an attribute of the attribute array. Note that we disable
// interrupts shortly, since the array is a volatile structure. An invalid index 
// is no operation.
//
//----------------------------------------------------------------------------------------
void setAttr( uint8_t index, uint16_t val ) {

    if ( index < MAX_DRV_ATTRIBUTES ) {

        cli( ); 
        drvAttributes[ index ] = val;
        sei( );
    }
}

//----------------------------------------------------------------------------------------
// "refreshAttr" is a function to refresh an attribute from the EEPROM content. An 
// invalid index is no operation.
//
//----------------------------------------------------------------------------------------
void refreshAttr( uint8_t index ) {

    if ( index < MAX_DRV_ATTRIBUTES ) {

        int ofs = sizeof( LcsDrvHeader ) + ( index * sizeof( uint16_t ));
        uint16_t tmp;
        
       EEPROM.get( ofs, tmp );

        cli( ); 
        drvAttributes[ index ] = tmp;
        cli( ); 
    }
}

//----------------------------------------------------------------------------------------
// "saveAttr" is a function to store an attribute to its EEPROM location. An invalid 
// index is no operation.
// 
//----------------------------------------------------------------------------------------
void saveAttr( uint8_t index ) {

    if ( index < MAX_DRV_ATTRIBUTES ) {

        int ofs = sizeof( LcsDrvHeader ) + ( index * sizeof( uint16_t ));

        cli( ); 
        uint16_t tmp = drvAttributes[ index ];
        cli( ); 

        EEPROM.put( ofs, tmp );
    }
}

//----------------------------------------------------------------------------------------
// The main routine to get the show going. A firmware layer is expected to call this 
// routine as the first thing. We will load the initial values from the EEPROM and 
// setup our I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t initDrvRuntime( ) {

  loadFromEEPROM( );
  initI2cChannel( );
 
  return( 0 );
}

//----------------------------------------------------------------------------------------
// The main loop. After the initialization, the firmware can perform its other setup
// task and the enter the runtime loop.
//
// ??? how do we call the firmware routines ?
//
// ??? we may just drop this and let the firmware provide the loop...
//----------------------------------------------------------------------------------------
void startDrvRuntime( ) {

   // setupWatchdog( ); // later ...

    while ( true ) {

      
    
    }
}





















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
