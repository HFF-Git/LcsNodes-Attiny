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
#include "LcsDrvAvrLib.h"

const uint8_t   DRV_MAJOR_VERSION   = 1;
const uint8_t   DRV_MINOR_VERSION   = 2;
const uint8_t   DRV_MAJOR_TYPE      = 1;
const uint8_t   DRV_MINOR_TYPE      = 2;

const uint16_t  DRV_VERSION         = ( DRV_MAJOR_VERSION << 8 ) | DRV_MINOR_VERSION;
const uint16_t  DRV_TYPE            = ( DRV_MAJOR_TYPE << 8 ) | DRV_MINOR_TYPE;

using namespace LCSDRV;


//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
void taskFunction( uint32_t *nextInterval ) {

    *nextInterval = 1000;  // in ms
}

//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
uint8_t requestFunction( uint8_t cmd, uint16_t *arg0, uint16_t *arg1 ) {

    switch ( cmd ) {

        case 64: { 

            return( 0 );
        
        } break;

        case 65: {

            *arg0 ^= 1;
            *arg1 ^= 1;
            return( 0 );
              
        } break;

        case 66: {

            return( 1 );
        }

        default: {

            return( 2 );
        }
    }
}


//----------------------------------------------------------------------------------------
//
//
//
//----------------------------------------------------------------------------------------
void setup( ) {

  
    uint8_t rStat = initDrvRuntime( DRV_TYPE, DRV_VERSION, 0 );
    if ( rStat != 0 ) drvFatalError( 8 );

    
    startDrvRuntime( taskFunction, requestFunction );
}

void loop( ) {

}
