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
#pragma once

#include "arduino.h"



//----------------------------------------------------------------------------------------
// The LcsBoardDesc structure defines what the board actually represents. It is 
// also the first structure that can be found on the controller board NVM as well 
// as the extension board controller, which acts like a NVM for that purpose. An 
// Atmega Attiny controller board also has the nice property of a serial number. 
// On the PICO, we "invent" a serial number. The header structure is 32 bytes 
// long and matches exactly what is used in the Attiny world.
//
// The options fields are board specific information. ??? explain what is there...
// 
// On the attiny, the header also hosts the current request parameters. A typical
// sequence will be to fill in the request fields and then periodically poll the
// status field for completion. 
//
// 
//----------------------------------------------------------------------------------------
struct LcsDrvBoardDesc {

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


//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
const uint16_t  DRV_MWORD           = 0xa5a5;

const int       MAX_DRV_ATTRIBUTES  = 64;

const uint8_t   DRV_CMD_ATTR_START  = 0;
const uint8_t   DRV_CMD_ATTR_END    = MAX_DRV_ATTRIBUTES - 1;

const uint8_t   DRV_CMD_REQ_START   = 64;
const uint8_t   DRV_CMD_REQ_END     = 127;

const uint8_t   DRV_CMD_IDLE        = 0xff;

//----------------------------------------------------------------------------------------
// I2C operations conceptually access a memory data structure. To the master the data
// structure just looks like an array of 16-bit words. The words are numbered from zero
// to header word size plus number of data words. The header is identical to the LcsNodes
// header used in the PICO controller world. 
//
// Parallel to the memmory structure is the EEPROM structure. The EEPROM structure, 
// initially created by a default formatting, is copied to the memory structure on 
// device reset. Updates to the memory can be synced to the EEPROM, some words in the
// memory structure are read-only.
//
//----------------------------------------------------------------------------------------
struct I2cMemData {

  LcsDrvBoardDesc head;                        // words  0 ... 15
  uint16_t     data[ MAX_DRV_ATTRIBUTES ];  // words 16 ... 79

};
