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
#include "LcsDrvAvrLib.h"


//=======================================================================================
//
//
//
//=======================================================================================
#include "arduino.h"
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LcsDrvAvrLib.h"

//=======================================================================================
// 4x4 matrix scan
//=======================================================================================

uint16_t raw_state;
uint16_t stable_state;
uint8_t  debounce[16];

//---------------------------------------------------------------------------------------
// Pin masks
//---------------------------------------------------------------------------------------

#define ROW_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)
#define COL_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm)

//---------------------------------------------------------------------------------------
// Initialization
//---------------------------------------------------------------------------------------

static void rows_init(void) {
    drvPinOutput(&PORTC, ROW_MASK);
    drvPinHigh(&PORTC, ROW_MASK);   // all inactive (HIGH)
}

static void cols_init(void) {
    drvPinInput(&PORTA, COL_MASK);
    drvPinPullup(&PORTA, COL_MASK);
}

//---------------------------------------------------------------------------------------
// Row selection
//---------------------------------------------------------------------------------------

static inline void select_row(uint8_t row) {
    drvPinHigh(&PORTC, ROW_MASK);                 // all HIGH
    drvPinLow(&PORTC, (PIN0_bm << row));          // one LOW
}

//---------------------------------------------------------------------------------------
// Matrix scan
//---------------------------------------------------------------------------------------

static uint16_t matrix_scan_raw(void) {
    uint16_t result = 0;

    for (uint8_t row = 0; row < 4; row++) {
        select_row(row);
        _delay_us(10);   // settle time

        // Read all columns at once
        uint8_t cols = ( ~ drvPortRead( &PORTA )) & COL_MASK;

        for (uint8_t col = 0; col < 4; col++) {
            if (cols & (PIN0_bm << col)) {
                result |= (1 << (row * 4 + col));
            }
        }
    }

    return result;
}

//---------------------------------------------------------------------------------------
// Debounce
//---------------------------------------------------------------------------------------

#define DEBOUNCE_TICKS 5

static void debounce_update(uint16_t raw) {
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t bit = (raw >> i) & 1;
        uint8_t stable = (stable_state >> i) & 1;

        if (bit == stable) {
            debounce[i] = 0;
        } else {
            if (++debounce[i] >= DEBOUNCE_TICKS) {
                stable_state ^= (1 << i);
                debounce[i] = 0;
            }
        }
    }
}

//---------------------------------------------------------------------------------------
// Arduino entry points
//---------------------------------------------------------------------------------------

void setup() {
    rows_init();
    cols_init();
    stable_state = 0;
}

void loop() {
    raw_state = matrix_scan_raw();
    debounce_update(raw_state);

    _delay_ms(10);
}
