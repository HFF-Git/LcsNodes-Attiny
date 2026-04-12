//========================================================================================
//
// LCS Driver Core Library - ATTINY version. 
//
//----------------------------------------------------------------------------------------
// The LCS driver core library for the ATTINY platform. This library is used by the 
//satellite boards for the LCS layout control system. The idea ist that the LCS Nodes
// provide an I2C interface to which the satellites are connected. The satellite board
// implements an I2C slave interface and provides a set of attributes and functions
// to the upper layer firmware. For example consider a turnout. Managing a modern
// turnout involves a lot of work. There is the turnout motor control, the feedback
// from the turnout position, the detection of a possible stall, etc. The satellite
// board can take care of all this work and provide a simple interface to the LCS node.
// 
// This library contains the core functionality and a set of routines to simplify
// satellite board software development. The idea is to handle as much as possible
// transparent to the firmware developer. For example, the I2C slave interface is 
// handled entirely at the core layer. The firmware layer just needs to check for
// incoming requests and write the results back to the respective memory locations. 
// 
// Like to the rest of the LCS software, there are a set of attributes which the 
// firmware programmer can use. They are are also backed by non-volatile memory. There
// are also routines for digital I/O, analog input, and a watchdog. The library also
// provides a routine to report fatal error conditions. In this case, the library 
// will blink the activity LED in a specific pattern.
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
#include <avr/interrupt.h>
#include <Wire.h>
#include <EEPROM.h>
#include "LcsDrvAvrLib.h"
#include "LcsDrvDesc.h"

//----------------------------------------------------------------------------------------
// Local name space. We keep here the local declarations and utility functions.
//
//----------------------------------------------------------------------------------------
namespace {

using namespace LCSDRV;

//----------------------------------------------------------------------------------------
// EEPROM. The EEPROM is used to store the attribute data. The first 16 bytes are 
// reserved for the header. The next 128 bytes are used for the attribute storage. 
// Each attribute is a 16-bit word. So, we can store 64 attributes in the EEPROM. The 
// first 8 are reserved for the core library itself.
//
//----------------------------------------------------------------------------------------
const uint32_t  EEPROM_MWORD            = 0xa5a5;
const uint16_t  EEPROM_HEADER_OFS       = 0;
const uint16_t  EEPROM_ATTR_RANGE_OFS   = 16;

//----------------------------------------------------------------------------------------
//
// Attribute - the first 8 are reserved. The words are identical to the LCS node nodeMap
// data structure.
// 
//  0 - boardType
//  1 - boardVersion
//  2 - serialNum1
//  3 - serialNum2
//  4 - serialNum3
//  5 - serialNum4
//  6 - boardOptions
//  
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
// The global variable for the timer. We use a timer interrupt to update the millisCount
// variable.
//
//----------------------------------------------------------------------------------------
volatile uint32_t millisCount = 0;

//----------------------------------------------------------------------------------------
// The attribute storage. It is used by the I2C layer to store and read data and by the
// access function for the upper firmware layer. It divides into the first eight locations
// reserved for the core library and the remaining 56 locations for the firmware.
//
//----------------------------------------------------------------------------------------
volatile uint16_t drvAttributes[ MAX_DRV_ATTRIBUTES ];

//----------------------------------------------------------------------------------------
// I2C data structures. We exchange data packets with 3 or 5 bytes. The master sends 
// a 3 byte data packet for writing attributes and a 5 byte packet for a request with
// two 16-bit arguments. This allows use to cleanly identify how to react to the write
// operation. A write of one byte starts a read request. The byte contains the command
// code which is used to decide of we return an attribute, a two byte answer, or the
// request data, which is a five byte answer consisting of status byte and two 16-bit
// arguments.  
//
// WRITE-ATTR:  i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H
// READ-ATTR:   i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H 
// 
// REQ-SEND: i2cAdr,i2cCommand,i2cArg0-L,i2cArg0-H,i2cArg1-L,i2cArg1-H
// REQ-RECV: i2cAdr,i2cCommand,i2cStatus,i2cArg0-L,i2cArg0-H,i2cArg1-L,i2cArg1-H
//
// Note that depending on the command code, the LCS node master expect to receive 
// the attribute data, the status and arguments of a request, or a simple status byte. 
// The firmware layer needs to fill the respective data structures and the I2C layer
// will take care of the rest. The firmware layer also needs to check for incoming 
// requests and handle them. The I2C layer will not wait for the firmware to handle 
// a request. It will just return the current status and argument data. So, the 
// firmware layer needs to periodically check for incoming requests and update the 
// status and argument data.
//
//----------------------------------------------------------------------------------------
uint8_t           i2cAdr;
volatile uint8_t  i2cWriteCmd;
volatile uint8_t  i2cReadCmd;
volatile uint8_t  i2cStatus;
volatile uint16_t i2cArg0;
volatile uint16_t i2cArg1;

//----------------------------------------------------------------------------------------
// "delayMs" is used by the fatal error code setting. When reporting a fatal error, as
// little as possible software should be used. Hence we have a minimalistic delay 
// routine. 
//
//----------------------------------------------------------------------------------------
void delayMs( uint16_t ms ) {
  
    while ( ms-- ) {
      
        // ~20,000 cycles per ms → 5000 iterations × ~4 cycles
        for ( uint16_t i = 0; i < 5000; i++ ) {
            __asm__ __volatile__( "nop" );
        }
    }
}

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
// Hash function for building a 64-bit value from the buffer input. This function is 
// used to build a unique 64-bit value from the serial number of the chip. We use the
// FNV-1a hash algorithm, which is simple and has good distribution properties. 
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
// value from that data using the FNV-1a hash function. This value is stored in the 
// attribute array and can be used by the firmware layer to identify the board. 
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
// The Attiny chip has a built in 32kHz oscillator for the RTC. We can use that to 
// measure the CPU frequency. Maybe a bit of an overkill, but this way we can be sure 
// that the CPU frequency is correctly detected and we can use it for the timer setup. 
// The routine sets up the RTC as a time base and the TCA0 timer as a free running counter.
// We wait for 1024 ticks of the RTC, which is about 31.25 ms, and then read the number 
// of ticks from the TCA0 timer. Since we know that 1024 ticks correspond to 1/32 of 
// a second, we can multiply the number of ticks by 32 to get the CPU frequency in Hz.
//
//----------------------------------------------------------------------------------------
uint32_t measureCpuFreq( void ) {

    while ( RTC.STATUS > 0 );

    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    RTC.CTRLA  = RTC_RTCEN_bm | RTC_PRESCALER_DIV1_gc;

    RTC.CNT = 0;

    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CNT   = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

    uint16_t start = RTC.CNT;
    while ((uint16_t)( RTC.CNT - start ) < 1024 );

    uint32_t ticks = TCA0.SINGLE.CNT;
    TCA0.SINGLE.CTRLA = 0;

    return ( ticks * 32 );
}

//----------------------------------------------------------------------------------------
// Setup the timer for the millis() function. We use TCA0 in single mode with a 
// pre-scaler of 64. The period is calculated based on the CPU frequency to achieve
// a 1 ms tick. We also enable the overflow interrupt to update the millisCount 
// variable. Note that we use a fixed CPU frequency for now, since the measurement
// can be a bit tricky and we want to get the timer running. Later, we can implement
// the CPU frequency measurement and use the actual value for the timer setup.
//
//----------------------------------------------------------------------------------------
void drvTimeInit( void ) {

    // uint32_t cpu_freq = measure_cpu_freq( );
    uint32_t cpu_freq = 20 * 1000 * 1000; // fix for now, later measure it.

    uint16_t prescaler = 64;
    uint16_t period    = ( cpu_freq / prescaler ) / 1000;
  
    TCA0.SINGLE.CTRLA   = 0; 
    TCA0.SINGLE.CTRB    = 0;
    TCA0.SINGLE.PER     = period; 
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA   = TCA_SINGLE_CLKSEL_DIV64_gc |
                          TCA_SINGLE_ENABLE_bm;

    sei( );
}

ISR( TCA0_OVF_vect ) {
  
    millisCount ++;
}

//----------------------------------------------------------------------------------------
// Setup the watchdog. The watchdog is used to reset the device in case of a hang or
// loop. We set it up with a period of about 8 seconds. The firmware layer needs to 
// periodically call the feedWatchdog() function to reset the watchdog timer. If the
// firmware fails to call the feedWatchdog() function within the specified period, 
// the watchdog will trigger a reset of the device. This is a safety mechanism to 
// ensure that the device can recover from a hang or loop condition. 
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
// EEPROM access. We have a set of routines to read and write 16-bit words to the EEPROM.
// The EEPROM is organized in bytes, so we need to read or write two bytes for each word.
// The routines check the word offset to ensure that we do not access out of bounds. 
// The read routine reads two bytes from the EEPROM and combines them into a 16-bit word.
// 
//----------------------------------------------------------------------------------------
void drvEepromReadWord( uint16_t wordOffset, uint16_t *value ) {
    
    if ( wordOffset >= ( EEPROM_SIZE / 2 )) return;

    const uint8_t *eeprom = (const uint8_t *)( EEPROM_START + ( wordOffset * 2 ));

    *value = eeprom[ 0 ] | ((uint16_t) eeprom[ 1 ] << 8 );
}

//----------------------------------------------------------------------------------------
// The EEPROM write routine is a bit more complex. We need to wait until the EEPROM is 
// ready before we can write. We also need to set the address and data registers before
// issuing the write command. Finally, we need to execute the write command atomically
// to avoid any interference from interrupts. We use the NVMCTRL_CMD_EEERWR_gc command
// to write a single byte to the EEPROM. We write the low byte and high byte separately,
// and only write a byte if it has changed to minimize wear on the EEPROM. Note that 
// the EEPROM has a limited number of write cycles, so we want to minimize unnecessary
// writes.
//
//----------------------------------------------------------------------------------------
#ifndef NVMCTRL_CMD_EEERWR_gc
#define NVMCTRL_CMD_EEERWR_gc 0x03
#endif

void eepromWriteByte( uint16_t addr, uint8_t value ) {
   
    while ( NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm );

    NVMCTRL.ADDR = addr;
    NVMCTRL.DATA = value;
    uint8_t sreg = SREG;
    cli( );
    CCP           = CCP_SPM_gc;
    NVMCTRL.CTRLA = NVMCTRL_CMD_EEERWR_gc;
    SREG          = sreg;
}

void drvEepromWriteWord( uint16_t wordOffset, uint16_t value ) {
  
    if ( wordOffset >= ( EEPROM_SIZE / 2 )) return;

    uint16_t addr = wordOffset * 2;
    const uint8_t *eeprom = (const uint8_t *)( EEPROM_START + addr );

    uint8_t low  = value & 0xFF;
    uint8_t high = value >> 8;

    if ( eeprom[ 0 ] != low  ) eepromWriteByte( addr, low );
    if ( eeprom[ 1 ] != high ) eepromWriteByte( addr + 1, high );
}

//----------------------------------------------------------------------------------------
// "formatEEPROM" creates a default memory structure and stores it to the EEPROM. This 
// function is typically called when the EEPROM is either brand new or was corrupted
// somehow. We build a default attribute array with the first eight locations reserved
// for the core library.
//
// ??? we do only write the magic word in the header. clear the rest ?
//----------------------------------------------------------------------------------------
void formatEEPROM( ) {

    uint64_t hwUID = buildHwUID( );

    drvAttributes[ 0 ] = DRV_TYPE;
    drvAttributes[ 1 ] = DRV_VERSION;
    drvAttributes[ 2 ] = hwUID & 0xFFFF; 
    drvAttributes[ 3 ] = ( hwUID >> 16 ) & 0xFFFF;
    drvAttributes[ 4 ] = ( hwUID >> 32 ) & 0xFFFF;
    drvAttributes[ 5 ] = ( hwUID >> 48 ) & 0xFFFF; 
    drvAttributes[ 7 ] = 0; 
  
    for ( int i = 8; i < MAX_DRV_ATTRIBUTES; i++ ) drvAttributes[ i ] = 0;
  
    #if 1
    EEPROM.put( EEPROM_HEADER_OFS, EEPROM_MWORD );
    EEPROM.put( EEPROM_ATTR_RANGE_OFS, drvAttributes, sizeof( drvAttributes ));
    #else   
    drvEepromWriteWord( EEPROM_HEADER_OFS, EEPROM_MWORD & 0xFFFF );
    drvEepromWriteWord( EEPROM_HEADER_OFS + 1, ( EEPROM_MWORD >> 16 ) & 0xFFFF );

    for ( int i = 0; i < MAX_DRV_ATTRIBUTES; i++ ) 
      drvEepromWriteWord( EEPROM_ATTR_RANGE_OFS + i, 0 );
    #endif
}

//----------------------------------------------------------------------------------------
// After reset, load the memory data structure with the content from the EEPROM. We 
// first read the EEPROM data and check if they mach the program defied constants. If
// valid, the remaining data is loaded from the EEPROM. If not, we format the EEPROM
// with default content. In either case, we return with a valid data setup.
// 
//----------------------------------------------------------------------------------------
void loadFromEEPROM( ) {

    uint32_t mWord;
    
    #if 1
    EEPROM.get( EEPROM_HEADER_OFS, mWord );
    #else
    uint16_t tmp;
    drvEepromReadWord( EEPROM_HEADER_OFS, &tmp );
    mWord = tmp;
    drvEepromReadWord( EEPROM_HEADER_OFS + 1, &tmp );
    mWord |= ( (uint32_t)tmp ) << 16;
    #endif

    if ( mWord != EEPROM_MWORD ) formatEEPROM( );

    #if 1
    EEPROM.get( EEPROM_ATTR_RANGE_OFS, drvAttributes, sizeof( drvAttributes ));
    #else
    for ( int i = 0; i < MAX_DRV_ATTRIBUTES; i++ )
        drvEepromReadWord( EEPROM_ATTR_RANGE_OFS + i, &drvAttributes[ i ]);
    #endif
}

//----------------------------------------------------------------------------------------
// The I2C channel receiver callback. We are informed that there is data on the I2C 
// channel. We need to read the data and react accordingly. The master can send three
// types of data packets. 
// 
// A packet of one byte is a read request. The byte contains
// the command code which we will use to decide what data to return. A packet of three
// bytes is a write to an attribute. The first byte is the command code which contains
// the attribute index. The next two bytes are the low and high byte of the attribute
// value. We need to check that the command code is in the attribute range and then
// update the attribute data. 
//
// Finally, a packet of five bytes is a request with two 16-bit arguments. The first
// byte is the command code which we will use to decide how to handle the request.
// The next four bytes are the low and high byte of the two arguments. We will store
// the command code and arguments in the respective global variables and let the upper
// layer firmware handle the request. 
// 
// Note that we do not wait for the firmware to handle the request. We just store
// the command code and arguments and return. The firmware layer needs to periodically
// check for incoming requests and handle them. 

//  GET: We send the following data:
//
//      W: i2cAdr, item
//      R: i2cAdr, arg-h, arg-l

//  SET: W: i2cAdr, item, arg-h, arg-l

//  REQ: We first send the request:
// 
//      W: i2cAdr, item, arg1-h, arg1-l, arg2-h, arg2-l
//
//  And then poll for the reply:
//
//      W: i2cAdr, item
//      R: i2cAdr, stat, arg1-h, arg1-l, arg2-h, arg2-l

//
//----------------------------------------------------------------------------------------
void receiveEvent( int numOfBytes ) {

    if ( numOfBytes == 0 ) {
        
        return;
    }
    else if ( numOfBytes == 1 ) {

        i2cReadCmd  = Wire.read( );
        i2cWriteCmd = DRV_CMD_IDLE;
    }
    else if ( numOfBytes == 3 ) {

        i2cReadCmd  = DRV_CMD_IDLE;
        i2cWriteCmd = Wire.read( );
    
        if ( i2cWriteCmd <= DRV_CMD_ATTR_END ) {

            uint8_t l = Wire.read( );
            uint8_t h = Wire.read( );
            drvAttributes[ i2cWriteCmd ] = ( l | ( h << 8 ));
        
            i2cReadCmd  = DRV_CMD_IDLE;
            i2cWriteCmd = DRV_CMD_IDLE;
        }
    }
    else if ( numOfBytes == 5 ) {

        i2cReadCmd  = DRV_CMD_IDLE;
        i2cWriteCmd = Wire.read( );
      
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
// The I2C channel master requests data. We need to return the requested data. The 
// master will typically loop reading the data until it gets the expected data. For 
// example, when the master sends a read request, it will loop reading the data until
// it gets the expected attribute value.
//
// Note that we cannot wait for the upper layer to finish the requested operation. When
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
// Setup the I2C channel. We use the I2C address from the attribute field "options".
// We also initialize the command and status variables. Finally, we register the receive
// and request event handlers. After this setup, the I2C channel is ready to handle
// incoming data and requests. The firmware layer just needs to check for incoming 
// requests and handle them. The I2C layer will take care of the rest.
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


//========================================================================================
//========================================================================================
// Library functions, externally visible.
//
//
//========================================================================================
//========================================================================================
namespace LCSDRV {

//========================================================================================
// Fatal Error Support
//
//
//========================================================================================
// We have one routine that will communicate a fatal error via the blinking of the 
// activity LED. The routine takes an integer as input, which is the error code. 
// The routine will blink the activity LED in a specific pattern to indicate the error
// code. The error code is typically a small integer, and the blinking pattern will
// consist of a series of short blinks followed by a long pause. For example, if the
// error code is 3, the LED will blink three times quickly, then pause for a longer
// period before repeating the pattern.
//
//----------------------------------------------------------------------------------------
void drvFatalError( int n ) {

    PORT_t  *port       = &PORTB;
    uint8_t pinBitMask  = 0;  // fix ...
    int     longPulse   = 1500;
    int     shortPulse  = 500;

    n = n % 8;
    port -> DIRSET = pinBitMask;

    while ( true ) {

        delayMs( longPulse );
       
        for ( int i = 0; i < n; i++ ) {

            
            port -> OUTSET = pinBitMask;
            delayMs( shortPulse );
            port -> OUTCLR = pinBitMask;
            delayMs( shortPulse );
        }
    }
}

//========================================================================================
// Basic Timer for timestamps and delays.
//
//
//========================================================================================
// The "drvMillis" function returns the number of milliseconds since the device was
// reset. The "drvDelay" function takes a number of milliseconds as input and blocks
// for that duration. We can use the built-in millis() and delay() functions from the
// Arduino library, or we can implement our own timer using the hardware timers. For now,
// we will use the built-in functions for simplicity. 
//
//----------------------------------------------------------------------------------------
uint32_t drvMillis( ) {

    #if 1
    return( millis( ));
    #else
    uint32_t m;
    uint8_t  sreg = SREG;
    cli( );
    m = millisCount;
    SREG = sreg;
    return ( m );
    #endif
}

void drvDelay( uint32_t val ) {

    #if 1
    delay( val );
    #else 
    uint32_t start = drvMillis( );
    while (( drvMillislis( ) - start ) < val );
    #endif   
}

//========================================================================================
// WatchDog Support
//
//
//========================================================================================
// The watchdog is used to restart the device on a hang or loop. If the software fails
// to refresh the watchdog within the specified period, the watchdog will trigger a
// reset of the device. 
//
//----------------------------------------------------------------------------------------
void feedWatchdog( ) {
  
    wdt_reset( );
}

bool wasWatchdogReset( ) {
    
    uint8_t flags = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = flags; 
    return ( flags & RSTCTRL_WDRF_bm );
}

//========================================================================================
// Digital Input/Output
//
//========================================================================================
// The following routines are our bread and butter routines for a digital IO pin. We
// generally use the PORT and port mask for naming a pin. The names are the AVR names
// for the ports and pins. For example, PORTB pin 5 would be referred to as PORTB, 
// PIN5_bm.
//
//----------------------------------------------------------------------------------------
void drvPinOutput( PORT_t *port, uint8_t pinBitmask ) {
    
    port -> DIRSET = pinBitmask;
}

void drvPinInput( PORT_t *port, uint8_t pinBitmask ) {
  
    port -> DIRCLR = pinBitmask;
}

void drvPinPullup ( PORT_t *port, uint8_t pinBitmask ) {

    for ( int i = 0; i < 8; i ++ ) {

        if ( pinBitmask & ( 1 << i )) {

            ( &port->PIN0CTRL )[ i ] = PORT_PULLUPEN_bm;
        }
    }
}

void drvPinWrite( PORT_t *port, uint8_t pinBitmask, uint8_t value ) {
  
    if ( value ) port -> OUTSET = pinBitmask;
    else         port -> OUTCLR = pinBitmask;
}

uint8_t drvPinRead( PORT_t *port, uint8_t pinBitmask )  {
    
    return ( drvPortRead( port ) & pinBitmask );
}

uint8_t drvPortRead( PORT_t *port ) {
  
    return ( port-> IN );
}
    
void drvPinHigh( PORT_t *port, uint8_t pinBitmask ) {
    
    port -> OUTSET = pinBitmask;
}

void drvPinLow( PORT_t *port, uint8_t pinBitmask ) {
    
    port -> OUTCLR = pinBitmask;
}

void drvPinToggle( PORT_t *port, uint8_t pinBitmask ) {
  
    port -> OUTTGL = pinBitmask;
}

//========================================================================================
// Analog Input
//
//
//========================================================================================
// The analog subsystem. During subsystem initialization, we need to configure the ADC
// hardware. We set the reference voltage, the clock prescaler, and enable the ADC 
// hardware.
//
// Note:
// - The corresponding pin must be configured for analog input (disable digital input buffer).
// - Unlike PORT pin configuration (bitmask-based), the ADC channel is selected
//   via an index using the MUXPOS field.
//
// Example:
//     uint16_t value = drvAnalogRead(ADC_MUXPOS_AIN3_gc);
//
// Conversion formula:
//     Vin = (adcValue / 1023.0) * Vref
//
// (Assumes 10-bit resolution. Adjust denominator if resolution changes.)
//
//----------------------------------------------------------------------------------------
void drvAdcInit( ) {
 
    VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc;
    ADC0.CTRLC = ADC_PRESC_DIV16_gc;
    ADC0.CTRLA = ADC_ENABLE_bm;

    for ( volatile int i = 0; i < 1000; i++ ); // delay a little
}

void drvAdcPinSetup(PORT_t *port, uint8_t pinBitmask ) {
  
    for ( uint8_t i = 0; i < 8; i++ ) {
      
        if ( pinBitmask & ( 1 << i )) {

            port->DIRCLR              = ( 1 << i );
            port->PIN0CTRL            |= PORT_PULLUPEN_bm;
            ( &port->PIN0CTRL ) [i ]  |= PORT_ISC_INPUT_DISABLE_gc;
        }
    }
}

uint16_t drvAnalogRead( uint8_t muxpos ) {
  
    ADC0.MUXPOS = muxpos;
    ADC0.COMMAND = ADC_STCONV_bm;

    while ( ! ( ADC0.INTFLAGS & ADC_RESRDY_bm ));
    
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    return ( ADC0.RES );
}

uint32_t adcToMilliVolts( uint16_t adcValue, uint32_t vrefMv ) {
  
    return ((uint32_t) adcValue * vrefMv ) / 1023;
}


//========================================================================================
// I2C Support
//
//========================================================================================
// At the heart of the satellite board is an I2C interface. 
//
// I2C Interface and Attribute Memory Model
//
// The satellite board is controlled via an I2C interface. Commands are received
// from the master and mapped to a central attribute memory structure. This memory
// acts as the single source of truth for all readable and writable data.
//
// - The I2C slave interrupt routines directly read from and write to this memory.
// - The firmware’s upper layer accesses the same data through defined APIs.
// - The attribute memory is persistently backed by corresponding EEPROM locations.
//
// Concurrency Note:
// The I2C slave operates in interrupt context, so concurrent access must be handled
// carefully. Any access from the main firmware must be synchronized to avoid data
// corruption.
//
// - Attribute data transfers are handled directly inside the I2C interrupt.
// - Function requests from the LCS node are not executed in the interrupt context.
//   Instead, they are stored in dedicated global request variables.
// - The main firmware loop periodically polls these variables, executes the
//   requested operation, and writes the result back to the same locations.
//
// This separation ensures that interrupt handling remains fast and deterministic,
// while more complex processing is deferred to the main execution context.
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

void i2cSetResponse( uint8_t rStat, uint16_t r0, uint16_t r1 ) {

    cli( );
    i2cStatus = rStat;
    i2cArg0   = r0;
    i2cArg1   = r1;
    sei( );
}

//========================================================================================
// Driver Attribute access.
//
//========================================================================================
// Attributes are a common concept in LCS. The satellite board also features a set of 
// attributes accessible to the firmware. There are 64 attributes. The first eight are
// reserved for the satellite library and contain information such as board type or 
// version. Attributes 8 to 63 are available to the firmware. Attributes are also 
// backed up by the EEPROM. There are routines to save or restore an attribute to or 
// from its EEPROM location. Note that the routines can also be called from within an 
// interrupt handler.
//
//----------------------------------------------------------------------------------------
uint16_t getAttr( uint8_t index ) {

    if ( index >=  MAX_DRV_ATTRIBUTES ) return( 0 );
   
    uint8_t sreg = SREG;
    cli( ); 
    uint16_t tmp = drvAttributes[ index ];
    SREG = sreg;

    return ( tmp );
}

//----------------------------------------------------------------------------------------
// A routine to update an attribute of the attribute array. Note that we disable
// interrupts shortly, since the array is a volatile structure. An invalid index 
// is no operation.
//
// ??? not all attributes are writable.
// ??? should we allow the setting, just not from remote ?
// ??? this way we could set board type and version, etc. programmatically in the 
// startup code.
//----------------------------------------------------------------------------------------
void setAttr( uint8_t index, uint16_t val ) {

    if ( index >= MAX_DRV_ATTRIBUTES ) return;

    if (( index >= 8 ) && ( index < MAX_DRV_ATTRIBUTES )) {
      
        uint8_t sreg = SREG;
        cli( );  
        drvAttributes[ index ] = val;
        SREG = sreg;
    }
    else {

        // ??? what cn we actually set ?


    }
}

//----------------------------------------------------------------------------------------
// "refreshAttr" is a function to load an attribute from its EEPROM location. An 
// invalid index is no operation. The entire attribute range is a valid index input.
// The routine reads the value from the EEPROM and updates the attribute array. Note
// that we disable interrupts shortly, since the array is a volatile structure. 
//
//----------------------------------------------------------------------------------------
void refreshAttr( uint8_t index ) {

    if ( index >= MAX_DRV_ATTRIBUTES ) return;

    int ofs = EEPROM_ATTR_RANGE_OFS + ( index * sizeof( uint16_t ));
    uint16_t tmp;
    
    EEPROM.get( ofs, tmp );

    uint8_t sreg = SREG;
    cli( );  
    drvAttributes[ index ] = tmp;
    SREG = sreg; 
}

//----------------------------------------------------------------------------------------
// "saveAttr" is a function to store an attribute to its EEPROM location. An invalid 
// index is no operation. The routine reads the value from the attribute array and 
// writes it to the EEPROM.  Note that we disable interrupts shortly, since the array
// is a volatile structure. 
//
//----------------------------------------------------------------------------------------
void saveAttr( uint8_t index ) {

    if ( index >= MAX_DRV_ATTRIBUTES ) return;

    int ofs = EEPROM_ATTR_RANGE_OFS + ( index * sizeof( uint16_t ));

    uint8_t sreg = SREG;
    cli( ); 
    uint16_t tmp = drvAttributes[ index ];
    SREG = sreg;

    EEPROM.put( ofs, tmp );
}

//========================================================================================
// Library core setup and loop.
//
//
//========================================================================================
// The main routine to get the show going. A firmware layer is expected to call this 
// routine as the first thing. We will load the initial values from the EEPROM and 
// setup our I2C channel. Between INIT and START of the runtime, a firmware can perform
// its own initialization work. The firmware's periodic work is a function that the 
// START routine will call periodically.
//
//----------------------------------------------------------------------------------------
uint8_t initDrvRuntime( uint16_t boardType, uint16_t boardVersion ) {

  loadFromEEPROM( );
  initI2cChannel( );
 
  return( 0 );
}

void startDrvRuntime( DriverFunction f ) {

   // setupWatchdog( ); // later ...

    while ( true ) {

      
    
    }
}























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

} // namespace LCSDRV
