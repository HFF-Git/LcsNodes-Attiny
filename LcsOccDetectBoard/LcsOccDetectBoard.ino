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






void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
