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

//----------------------------------------------------------------------------------------
// The magic word. A confugured EEPROM needs to have as the first two bytes this number.
//
//----------------------------------------------------------------------------------------
  const uint16_t  EEPROM_MWORD            = 0xa5a5;
  const uint16_t  EEPROM_MWORD_OFS        = 0;
  const uint16_t  EEPROM_ATTR_RANGE_OFS   = 8;

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
//
// ??? rethink this. We need to check the magic word. Fine.
// ??? the board type and version could also be the first two attributes. Easier to 
// read this way. 
// ??? the serial number is produced on request. No need to store.
// ??? the boardOptions could be actually two words, one for the library and one 
// for the firmware. Both are also atributes. The library word is readonly and 
// can only be modifed by the configuratio process. The firmware options word
// is passed through the init runtime.
//
// ??? the EEPROM would then mirror the attributes as before. BUT there is no
// header in that sense.
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
//
// Attribute - the first 8 are reserved. Leaves 56 for the firmware.
//
//  0 - boardType
//  1 - boardVersion
//  2 - serialNum1
//  3 - serialNum2
//  4 - serialNum3
//  5 - serialNum4
//  6 - boardOptions
//  7 - firmwareOptions
//
//----------------------------------------------------------------------------------------

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
// "delayMs" is used by teh fatal error code setting. When reporting a fatal error, as
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
// Setup a basic timer for the timestamp functions.
//
// 
//
// ??? need to set up a basic timer for the micro/millis function.
//----------------------------------------------------------------------------------------





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

  tmp.boardMword       = EEPROM_MWORD;                   
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
    if (( drvHeader.boardMword != EEPROM_MWORD ) ||
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


//========================================================================================
//========================================================================================
// Library functions, externally visible.
//
//
//========================================================================================
//========================================================================================


//========================================================================================
// Fatal Error Support
//
//
//========================================================================================

//----------------------------------------------------------------------------------------
//
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

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
volatile uint32_t millisCount = 0;

void drvTimeInit( void ) {
  
    TCA0.SINGLE.CTRLA     = TCA_SINGLE_CLKSEL_DIV64_gc;
    // TCA0.SINGLE.PER       = 250; // ~1ms at 16MHz / 64
    TCA0.SINGLE.PER       = 313; // ~1ms at 20MHz / 64
    TCA0.SINGLE.INTCTRL   = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA     |= TCA_SINGLE_ENABLE_bm;

    sei( );
}

ISR( TCA0_OVF_vect ) {
  
    millisCount ++;
}

uint32_t drvMillis( ) {

    #if 1
    return( millis( ));
    #else
    uint32_t m;
    cli( );
    m = millisCount;
    sei( );
    return m;
    #endif
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
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

//========================================================================================
// Digital Input/Output
//
//
//========================================================================================

//----------------------------------------------------------------------------------------
//
// Example. drvPinMode( &PORTB, PIN5_bm ) 
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

uint8_t drvPinRead( PORT_t *port, uint8_t pinBitmask ) {
  
    return (( port ->IN & pinBitmask ) != 0 );
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

//----------------------------------------------------------------------------------------
// Seetup the ADC subsystem. We set the reference voltage and clock prescaler and finally
// enable the ADC hardware.
//
// ??? how to set it for our RailCOm scenario ?
//----------------------------------------------------------------------------------------
void drvAdcInit( ) {
 
    VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc;
    ADC0.CTRLC = ADC_PRESC_DIV16_gc;
    ADC0.CTRLA = ADC_ENABLE_bm;

    for ( volatile int i = 0; i < 1000; i++ ); // delay a little
}

//----------------------------------------------------------------------------------------
// Configure a pin for analog input. We are passed the port and port bitmask.
// 
// Example.drvAdcPinSetup( &PORTA, PIN3_bm );
//
//----------------------------------------------------------------------------------------
void drvAdcPinSetup(PORT_t *port, uint8_t pinBitmask ) {
  
    for ( uint8_t i = 0; i < 8; i++ ) {
      
        if ( pinBitmask & ( 1 << i )) {

            port->DIRCLR              = ( 1 << i );
            port->PIN0CTRL            |= PORT_PULLUPEN_bm;
            ( &port->PIN0CTRL ) [i ]  |= PORT_ISC_INPUT_DISABLE_gc;
        }
    }
}

//----------------------------------------------------------------------------------------
// Read analog input. In contrast to the port bitmap used for the pin, the analog hardware
// is simply indexed to select the channel.
//
// Example: uint16_t value = drvAnalogRead( ADC_MUXPOS_AIN3_gc );
//
// Actual Voltage: Vin = ( adcValue / 1023 )  * Vref
//
//----------------------------------------------------------------------------------------
uint16_t drvAnalogRead( uint8_t muxpos ) {
  
    ADC0.MUXPOS = muxpos;
    ADC0.COMMAND = ADC_STCONV_bm;

    while ( ! ( ADC0.INTFLAGS & ADC_RESRDY_bm ));
    
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    return ( ADC0.RES );
}

//----------------------------------------------------------------------------------------
// A little helper function to convert adc digits to a voltage.
//
//----------------------------------------------------------------------------------------
uint32_t adcToMilliVolts( uint16_t adcValue, uint32_t vref_mV ) {
  
    return ((uint32_t) adcValue * vref_mV ) / 1023;
}


//========================================================================================
// I2C Support
//
//========================================================================================
// At the heart of the satellite board is an I2C interface. Commands are sent to the 
// satllite board, data is rwad or written to the attribute map. The implementation 
// features a cenzral mempory data structure. The I2C slave routines store and retrieve
// data from this data structure, the fimrware upper layer accesses this memory via 
// defined procedures. Finally, this memory is backed by repsective EEPROM locations.
// 
// Care has to be taken when accessing the memory. The I2C slave interface runs as an
// interrupt routine. So memory access from teh upper layer needs to be synchornized. 
// Data, i.e. attribute access, is pwerformed directly in the I2C interrupt handler.
// A function request from the LCS node is stored in global variables. Teh firmware
// layer periodcally polls these locations. When there is a request, it is handled
// and the result written back to the request locations.
//
//----------------------------------------------------------------------------------------

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

//========================================================================================
// Driver Attribute access.
//
//========================================================================================
// Attributes are a common concept in LCS. The satellite board also features a set of 
// attributes accessible to the firmware. There are 64 attribites. The first eight are
// reserved for the satellite libray and contains information such as board type or 
// version. Attributes 8 to 63 are available to the firmeware. Attributes are also 
// bac up by the EEPROM. There are routines to save or restore an attribute to or from
// its EEPROM location.
//
//----------------------------------------------------------------------------------------

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
// ??? not all attributes are writable.
// ??? should we allow the setting, just not from remote ?
// ??? this way we could set boardtype and version, etc. programatically in the 
// startup code.
//----------------------------------------------------------------------------------------
void setAttr( uint8_t index, uint16_t val ) {

    if ( index >= MAX_DRV_ATTRIBUTES ) return;

    if (( index >= 8 ) && ( index < MAX_DRV_ATTRIBUTES )) {
      
        cli( ); 
        drvAttributes[ index ] = val;
        sei( );
    }
}

//----------------------------------------------------------------------------------------
// "refreshAttr" is a function to refresh an attribute from the EEPROM content. An 
// invalid index is no operation. The entire attribute range is a valid index input.
// 
//----------------------------------------------------------------------------------------
void refreshAttr( uint8_t index ) {

    if ( index < MAX_DRV_ATTRIBUTES ) {

        int ofs = EEPROM_ATTR_RANGE_OFS + ( index * sizeof( uint16_t ));
        uint16_t tmp;
        
       EEPROM.get( ofs, tmp );

        cli( ); 
        drvAttributes[ index ] = tmp;
        cli( ); 
    }
}

//----------------------------------------------------------------------------------------
// "saveAttr" is a function to store an attribute to its EEPROM location. An invalid 
// index is no operation. The entire attribute range is a valid index input.
// 
// ??? where do we check remote access to 0 .. 7 ?
//----------------------------------------------------------------------------------------
void saveAttr( uint8_t index ) {

    if ( index < MAX_DRV_ATTRIBUTES ) {

        int ofs = EEPROM_ATTR_RANGE_OFS + ( index * sizeof( uint16_t ));

        cli( ); 
        uint16_t tmp = drvAttributes[ index ];
        cli( ); 

        EEPROM.put( ofs, tmp );
    }
}

//=======================================================================================
// Servo support.
//
//
//=======================================================================================
// LCS features a servo library. Up to eight services are managed by the drivr library.
// Each servo has a lower and upper limit for the servo position. IN addition, there
// is a time value how long it should take to go from a current position to the target
// position. 
//
// Servos are nicely staggered in the time slot. As each servo ecpects ot be refreshed
// 50 times a second, we place the up to eight servos in the 20ms slot one after the 
// other in their own 2.5 millsecod slot. Hence eight sevos max. 
//
// The servo subsysten has its onw table with the configuration data. Typically, the 
// three values lower, upper and time are defined in the attribute range and synched
// with this memory structure.
//
// ??? idea is to lower the total current consumption. 
// ??? we could also have an attribute with "position".
//----------------------------------------------------------------------------------------
#define SERVO_COUNT   8
#define SERVO_SLOT_US 2500
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

//----------------------------------------------------------------------------------------
// The servo configuration data. We have the HW pin, the upper and lower limit and a
// transition time, which specifies how ling it will take going from current position 
// to the target position. 
//
//----------------------------------------------------------------------------------------
struct ServoConfig {

    PORT_t   port;
    uint8_t  pinBitMask;
    uint16_t lower;
    uint16_t upper;
    uint16_t transitionMs;
};

//----------------------------------------------------------------------------------------
// The servo current state data.
//
//----------------------------------------------------------------------------------------
struct ServoState {
  
    uint16_t current;
    uint16_t target;
    uint16_t start;
    uint32_t t0;
};


//----------------------------------------------------------------------------------------
// Global variables for servo support.
//
//----------------------------------------------------------------------------------------
uint8_t           activeServoCount;
ServoConfig       servoConfig[ SERVO_COUNT ];
ServoState        servoState[ SERVO_COUNT ];
volatile uint8_t  activeServo;
volatile bool     activeServoHighPhase;
volatile uint16_t servoPulseWidth[ SERVO_COUNT ];


//----------------------------------------------------------------------------------------
// Initialize the servo subsystenm.
//
// ??? pass the congig arrray instead of the pins ?
// ??? pass attribute index and store all this data in attributes in the first place ?
//----------------------------------------------------------------------------------------
uint8_t servoSetupServoSubsys( int numOfServos, ServoConfig *cfg ) {
  
    if ( numOfServos > SERVO_COUNT ) return 99;

    activeServoCount      = numOfServos;
    activeServo           = 0;
    activeServoHighPhase  = false;

    // Copy configuration
    for (uint8_t i = 0; i < numOfServos; i++) {

        servoConfig[i] = cfg[i];
        drvPinOutput( &servoConfig[i].port, servoConfig[i].pinBitMask);
    }

    // Initialize state
    for (uint8_t i = 0; i < numOfServos; i++) {

        servoState[i].current = servoConfig[i].lower;
        servoState[i].target  = servoConfig[i].lower;
        servoState[i].start   = servoState[i].current;
        servoState[i].t0      = millis();

        servoPulseWidth[i] = servoState[i].current;
    }
    

    // Timer setup (TCB0)
    cli();

    TCB0.CTRLA    = 0;
    TCB0.CTRLB    = TCB_CNTMODE_INT_gc;
    TCB0.CCMP     = 1000;
    TCB0.INTCTRL  = TCB_CAPT_bm;
    TCB0.CTRLA    = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;

    sei();

    return 0;
}

//----------------------------------------------------------------------------------------
// 
//
//----------------------------------------------------------------------------------------
static inline void servoSetTimer( uint16_t us ) {
  
    uint16_t ticks = ( F_CPU / 2000000UL ) * us; // F_CPU/2 → 0.5µs
    TCB0.CCMP = ticks;
}

//----------------------------------------------------------------------------------------
// Update the servo state.
//
// ??? !!! needs to be called periodcally from outer loop.
//----------------------------------------------------------------------------------------
void servoUpdate( ) {
  
    uint32_t now = drvMillis();

    for (uint8_t i = 0; i < activeServoCount; i++) {

        auto &s = servoState[i];
        auto &c = servoConfig[i];

        if (s.current == s.target) continue;

        uint32_t dt = now - s.t0;

        if (dt >= c.transitionMs || c.transitionMs == 0) {
            s.current = s.target;
        }
        else {
            int32_t diff = (int32_t)s.target - s.start;
            s.current = s.start + diff * dt / c.transitionMs;
        }

        servoPulseWidth[i] = s.current;
    }
}

//----------------------------------------------------------------------------------------
// The servo interrupt handler code. 
// 
//----------------------------------------------------------------------------------------
void servoIsrHandler( ) {
  
    static bool    highPhase  = false;

    uint16_t pw = servoPulseWidth[ activeServo ];

    if (pw < SERVO_MIN_US) pw = SERVO_MIN_US;
    if (pw > SERVO_MAX_US) pw = SERVO_MAX_US;

    if ( highPhase ) {

        activeServoHighPhase = false;
      
        // End pulse
        drvPinLow( &servoConfig[activeServo].port, servoConfig[activeServo].pinBitMask );
        servoSetTimer( SERVO_SLOT_US - pw );
        
        // Move to next servo after full cycle
        activeServo++;
        if ( activeServo >= activeServoCount ) activeServo = 0;
    }
    else {

        activeServoHighPhase = true;
        
        // Start pulse
        drvPinHigh(&servoConfig[activeServo].port, servoConfig[activeServo].pinBitMask);
        servoSetTimer( pw );
    }
}

//----------------------------------------------------------------------------------------
// The actual interrupt routine.
//
//----------------------------------------------------------------------------------------
ISR( TCB0_INT_vect ) {

    TCB0.INTFLAGS = TCB_CAPT_bm; // clear interrupt flag
    servoIsrHandler( );
}


//----------------------------------------------------------------------------------------
// Set a servo position within the bounds of lower and upper config value. We set the
// new target in the servo state and also remember the time we did it.
//
// ??? 8-bit value ?
//----------------------------------------------------------------------------------------
static inline uint16_t servoMap(uint8_t v, uint16_t lower, uint16_t upper)
{
    return lower + ((uint32_t)(upper - lower) * v) / 255;
}

void servoSet(uint8_t i, uint8_t value)
{
    if (i >= activeServoCount) return;

    if (value > 255) value = 255;

    uint16_t mapped = servoMap(value,
                               servoConfig[i].lower,
                               servoConfig[i].upper);

    servoState[i].target = mapped;
    servoState[i].start  = servoState[i].current;
    servoState[i].t0     = millis();
}




















//========================================================================================
// Library core setup and loop.
//
//
//========================================================================================

//----------------------------------------------------------------------------------------
// The main routine to get the show going. A firmware layer is expected to call this 
// routine as the first thing. We will load the initial values from the EEPROM and 
// setup our I2C channel.
//
//----------------------------------------------------------------------------------------
uint8_t initDrvRuntime( uint16_t boardType, uint16_t boardVersion ) {

  loadFromEEPROM( );
  initI2cChannel( );
 
  return( 0 );
}

//----------------------------------------------------------------------------------------
// The main loop. After the initialization, the firmware can perform its other setup
// task and the enter the runtime loop.
//
// 
//
// ??? we do the loop here but breakout to the driver code...
//----------------------------------------------------------------------------------------
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
