//========================================================================================
//
//
//
//----------------------------------------------------------------------------------------
// 
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


//----------------------------------------------------------------------------------------
// The LcsBoardDesc structure defines what the board actually represents. It is 
// also the first structure that can be found on the controller board NVM as well 
// as the extension board controller, which acts like a NVM for that purpose. An 
// Atmega Attiny controller board also has the nice property of a serial number. 
// We use it for I2C bus collision detection. On the PICO, we "invent" a serial 
// number. The header structure is 32 bytes long and matches exactly what is 
// used in the Attiny world.
//
//----------------------------------------------------------------------------------------
struct LcsBoardDesc {

    uint32_t            boardMword;                     // magic word
    uint16_t            boardInfo;                      // type/subtype
    uint16_t            boardCtrlInfo;                  // family / cType
    uint16_t            boardVersion;                   // major / sub version
    uint16_t            serialNum1;                     // serial number part 1
    uint16_t            serialNum2;                     // serial number part 2
    uint16_t            serialNum3;                     // serial number part 3
    uint16_t            serialNum4;                     // serial number part 4  
    uint16_t            boardOptions;                   // board config options
    uint16_t            boardI2cAdr;                    // board I2C address       
    uint16_t            boardStatus;                    // board status bits
    uint16_t            boardCommand;                   // board command 
    uint16_t            boardNumOfRegs;                 // board registers          
    uint16_t            reserved1;                      
    uint16_t            reserved2;  
};
