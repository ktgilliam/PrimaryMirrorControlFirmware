/*******************************************************************************
Copyright 2022
Steward Observatory Engineering & Technical Services, University of Arizona
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

/**
@brief Global Definitions used by all files 
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror_golbal.h

*/



#ifndef PRIMARY_MIRROR_GLOBAL_H
#define PRIMARY_MIRROR_GLOBAL_H

#define PMC_LABEL "LFAST PRIMARY MIRROR CONTROL"

//#define TEENSY41

#define A_STEP 2
#define A_DIR  5
#define A_LIMIT_SW_PIN  9

#define B_STEP 3
#define B_DIR  6
#define B_LIMIT_SW_PIN 10

#define C_STEP 4
#define C_DIR  7
#define C_LIMIT_SW_PIN 11

#define STEP_ENABLE_PIN 8 
#define FAN_CONTROL 0     // Unconfirmed

#define LED_PIN 13

#define SW 37
#define VRx 38
#define VRy 39

//Determine Network values
#define MAC { 0x00, 0x50, 0xB6, 0xEA, 0x8F, 0x44 }
// #define IPAdd   { 169,254,232,24 }
//#define MAC     { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }
#define IPAdd   {192, 168, 121, 177}
#define GATEWAY 0,0,0,0
#define SUBNET  0,0,0,0
#define PORT    4500

#define UPDATE_PRD_US 100
#define TERM_UPDATE_PRD_SEC 0.2
constexpr uint32_t TERM_UPDATE_COUNT = TERM_UPDATE_PRD_SEC / (UPDATE_PRD_US * 1e-6);
#define MIRROR_RADIUS 281880  // Radius of mirror actuator positions in um 
#define STEPPER_MAX_SPEED 800.0
#define STEPPER_MAX_ACCEL 100.0

constexpr uint32_t EEPROM_ADDR_START = 0;
constexpr uint32_t EEPROM_ADDR_STEPPER_A_POS = (EEPROM_ADDR_START + 0);
constexpr uint32_t EEPROM_ADDR_STEPPER_B_POS = (EEPROM_ADDR_STEPPER_A_POS + sizeof(uint32_t));
constexpr uint32_t EEPROM_ADDR_STEPPER_C_POS = (EEPROM_ADDR_STEPPER_B_POS + sizeof(uint32_t));
constexpr uint32_t EEPROM_ADDR_IS_HOMED = (EEPROM_ADDR_STEPPER_C_POS + sizeof(uint32_t));
constexpr uint32_t EEPROM_ADDR_RESET_NOTIFIER = (EEPROM_ADDR_IS_HOMED + sizeof(uint32_t));

#define ENABLE_TERMINAL_UPDATES 1

#endif