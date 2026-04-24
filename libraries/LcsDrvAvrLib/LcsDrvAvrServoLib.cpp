//========================================================================================
//
// 
//
//----------------------------------------------------------------------------------------
// LCS features a servo library. Up to eight services are managed by the driver library.
// Each servo has a lower and upper limit for the servo position. IN addition, there
// is a time value how long it should take to go from a current position to the target
// position. 
//
// Servos are nicely staggered in the time slot. As each servo expects to be refreshed
// 50 times a second, we place the up to eight servos in the 20ms slot one after the 
// other in their own 2.5 millisecond slot. Hence eight servos max. 
//
// The servo subsystem has its on table with the configuration data. Typically, the 
// three values lower, upper and time are defined in the attribute range and synched
// with this memory structure.
//
// ??? idea is to lower the total current consumption. 
// ??? we could also have an attribute with "position".
//
// ??? is this a good idea to keep this outside of the AVR library ? 
// ??? we could also have a flag that says we use servos or not. 
// ??? the issue might only be that we use a dedicated timer. 
// ??? bit then we can keep the main loop inside and take care of all work that needs
// to be done ...
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
#include <avr/interrupt.h>
#include "LcsDrvAvrLib.h"

using namespace LCSDRV;

//----------------------------------------------------------------------------------------
// Local name space. We keep here the local declarations and utility functions.
//
//----------------------------------------------------------------------------------------
namespace {

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
#define SERVO_COUNT   8
#define SERVO_SLOT_US 2500
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define SERVO_US_MIN 1000
#define SERVO_US_MAX 2000


  
}

//=======================================================================================
// Servo support.
//
//
//=======================================================================================



//----------------------------------------------------------------------------------------
// The servo configuration data. We have the HW pin, the upper and lower limit and a
// transition time, which specifies how ling it will take going from current position 
// to the target position. 
//
//----------------------------------------------------------------------------------------
struct ServoConfig {

    PORT_t    *port;
    uint8_t   pinBitMask;
    uint8_t   lower;
    uint8_t   upper;
    uint16_t  transitionMs;
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
//
//
//----------------------------------------------------------------------------------------
uint16_t mapToUs( uint8_t v ) {
    
    return ( SERVO_US_MIN + ((uint32_t)( SERVO_US_MAX - SERVO_US_MIN ) * v ) / 255 );
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void servoSetupTimer( ) {

    // Timer setup (TCB0)
    uint8_t sreg = SREG;
    cli( ); 

    TCB0.CTRLA    = 0;
    TCB0.CTRLB    = TCB_CNTMODE_INT_gc;
    TCB0.CCMP     = 1000;
    TCB0.INTCTRL  = TCB_CAPT_bm;
    TCB0.CTRLA    = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;

    SREG = sreg;
}

//----------------------------------------------------------------------------------------
// Initialize the servo subsystem.
//
// 
// ??? pass attribute index and store all this data in attributes in the first place ?
// ??? not so easy for port and mask. 
// ??? how about we have a config array as planned.
// ??? and also add a parameter at which attribute offset the data is stored.
//----------------------------------------------------------------------------------------
uint8_t servoSetupServoSubsys( int numOfServos, ServoConfig *cfg ) {
  
    if ( numOfServos > SERVO_COUNT ) return 99;

    activeServoCount      = numOfServos;
    activeServo           = 0;
    activeServoHighPhase  = false;

    // Copy configuration
    for ( uint8_t i = 0; i < numOfServos; i++ ) {

        servoConfig[i] = cfg[i];
        drvPinOutput( servoConfig[i].port, servoConfig[i].pinBitMask);
    }

    // Initialize state
    for ( uint8_t i = 0; i < numOfServos; i++ ) {

        servoState[i].current = mapToUs(servoConfig[i].lower);
        servoState[i].target  = servoState[i].current;
        servoState[i].start   = servoState[i].current;
        servoState[i].t0      = drvMillis();

        servoPulseWidth[i] = servoState[i].current;
    }
    

    // Timer setup (TCB0)
    servoSetupTimer( );
    
    return 0;
}

//----------------------------------------------------------------------------------------
// 
//
// ??? need to get the CPU frequency dynamically....
//----------------------------------------------------------------------------------------
void servoSetTimer( uint16_t us ) {
  
    uint16_t ticks = ( F_CPU / 2000000UL ) * us; // F_CPU/2 → 0.5µs
    TCB0.CCMP = ticks;
}

//----------------------------------------------------------------------------------------
// The servo interrupt handler code. 
// 
//----------------------------------------------------------------------------------------
void servoIsrHandler( ) {
  
    uint16_t pw = servoPulseWidth[ activeServo ];

    if (pw < SERVO_MIN_US) pw = SERVO_MIN_US;
    if (pw > SERVO_MAX_US) pw = SERVO_MAX_US;

    if ( activeServoHighPhase ) {

        activeServoHighPhase = false;
      
        // End pulse
        drvPinLow( servoConfig[activeServo].port, servoConfig[activeServo].pinBitMask );
        servoSetTimer( SERVO_SLOT_US - pw );
        
        // Move to next servo after full cycle
        activeServo++;
        if ( activeServo >= activeServoCount ) activeServo = 0;
    }
    else {

        activeServoHighPhase = true;
        
        // Start pulse
        drvPinHigh( servoConfig[activeServo].port, servoConfig[activeServo].pinBitMask);
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
// Update the servo state.
//
// ??? !!! needs to be called periodically from outer loop.
// ??? is this also a good place to keep attributes and config in sync ?
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

        uint8_t sreg = SREG;
        cli();
        servoPulseWidth[i] = s.current;
        SREG = sreg;
    }
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void servoSet( uint8_t i, uint8_t value ) {
  
    if ( i >= activeServoCount ) return;

    auto &cfg = servoConfig[ i ];
    auto &st  = servoState[ i ];

    // Clamp to configured limits
    if ( value < cfg.lower ) value = cfg.lower;
    if ( value > cfg.upper ) value = cfg.upper;

    // Convert to pulse width
    uint16_t targetUs = mapToUs( value );

    st.target = targetUs;
    st.start  = st.current;
    st.t0     = drvMillis();
}

// for debugging / calibration.
void servoSetUs(uint8_t i, uint16_t us )
{
    if (i >= activeServoCount) return;

    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    auto &st = servoState[i];

    st.target = us;
    st.start  = st.current;
    st.t0     = drvMillis();
}

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
void servoSetMin(uint8_t i) { servoSet( i, 0 ); }
void servoSetMax(uint8_t i) { servoSet( i, 255 ); }
