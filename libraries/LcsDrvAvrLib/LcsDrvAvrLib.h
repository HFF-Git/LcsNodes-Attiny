//========================================================================================
//
// LCS Driver Core Library - ATTINY version. 
//
//----------------------------------------------------------------------------------------
// 
//
//
//----------------------------------------------------------------------------------------
//
// Layout Control System - Runtime Library internals include file
// Copyright (C) 2026 - 2026 Helmut Fieres
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

namespace LCSDRV {

typedef void ( *DriverFunction )( void );

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
const uint8_t   MAX_DRV_ATTRIBUTES  = 64;

//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
enum DrvCmdItems : uint8_t {

  DRV_CMD_ATTR_START  = 0,
  DRV_CMD_ATTR_END    = MAX_DRV_ATTRIBUTES - 1,

  DRV_CMD_REQ_START   = 64,
  DRV_CMD_REQ_END     = 127,

  DRV_CMD_IDLE        = 0xff
};


//----------------------------------------------------------------------------------------
//
//
//----------------------------------------------------------------------------------------
enum DrvErrIds : uint8_t {

  DRV_NO_ERR = 0

  
};

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
void      drvFatalError( int n );
uint32_t  drvMillis( );
void      drvDelay( uint32_t val );

void      feedWatchdog( );
bool      wasWatchdogReset( );

void      drvPinOutput( PORT_t *port, uint8_t pinBitmask );
void      drvPinInput( PORT_t *port, uint8_t pinBitmask );
void      drvPinPullup ( PORT_t *port, uint8_t pinBitmask );
void      drvPinWrite( PORT_t *port, uint8_t pinBitmask, uint8_t value );
uint8_t   drvPinRead( PORT_t *port, uint8_t pinBitmask );
uint8_t   drvPortRead(  PORT_t *port );
void      drvPinHigh( PORT_t *port, uint8_t pinBitmask );
void      drvPinLow( PORT_t *port, uint8_t pinBitmask );
void      drvPinToggle( PORT_t *port, uint8_t pinBitmask );

int       i2cGetRequest( uint8_t *cmd, uint16_t *a0, uint16_t *a1 );
void      i2cSetResponse( uint8_t rStat, uint16_t r0, uint16_t r1 );

uint16_t  getAttr( uint8_t index );
void      setAttr( uint8_t index, uint16_t val );
void      refreshAttr( uint8_t index );
void      saveAttr( uint8_t index );

uint8_t   initDrvRuntime( uint16_t boardType, 
                          uint16_t boardVersion,
                          uint16_t firmwareOptions );
void      startDrvRuntime( );

} // namespace LCSDRV
