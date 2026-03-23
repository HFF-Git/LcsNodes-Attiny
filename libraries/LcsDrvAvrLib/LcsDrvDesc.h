//========================================================================================
// 
//  Board Descriptor file.
//
//----------------------------------------------------------------------------------------
// Each satellite board has a type and sub type, a version and sub version. The library
// stores this data on the EEPROM when the EEPROM is formatted. 
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

// ??? this rather belongs to an actual board project .... ?

#pragma once

const uint8_t   DRV_MAJOR_VERSION   = 1;
const uint8_t   DRV_MINOR_VERSION   = 2;
const uint8_t   DRV_MAJOR_TYPE      = 3;
const uint8_t   DRV_MINOR_TYPE      = 4;

const uint16_t  DRV_VERSION         = ( DRV_MAJOR_VERSION << 8 ) | DRV_MINOR_VERSION;
const uint16_t  DRV_TYPE            = ( DRV_MAJOR_TYPE << 8 ) | DRV_MINOR_TYPE;
