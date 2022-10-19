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
@brief LFAST prototype Primary Mirror Control Interface firmware
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror.cpp

This file contains the firmware code for the LFAST Prototype Primary 
Mirror Control interface. 
*/

#include "primary_mirror_global.h"
#include "primary_mirror_ctrl.h"
#include "primary_mirror_network.h"

// Parsing of JSON style command done in network file, for now.

/*
Features: 
Manual Control: Joystick, Arrow Keys

To Do:
What are desired max speeds of Stepper Motors?

Network Setup
  - JSON Style command parsing

Threading
  - 'TeensyThreads.h'
  - Activate function as thread to allow other function to take over if needed

System Recovery
  - Remember positions of stepepr motors in case of ungraceful shutdown. (EEPROM)
*/

// Figure out appropriate pin outs
//AccelStepper name(mode, step_pin, direction_pin)
AccelStepper tip(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper tilt(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper focus(AccelStepper::DRIVER, Z_STEP, Z_DIR);

void setup() {
  Serial.begin(9600);

  tip.setMaxSpeed(800.0); // Steps per second
  tip.setAcceleration(100.0); // Steps per second per second
  pinMode(X_LIM, INPUT_PULLUP);

  tilt.setMaxSpeed(800.0);
  tilt.setAcceleration(100.0);
  pinMode(Y_LIM, INPUT_PULLUP);

  focus.setMaxSpeed(800.0);
  focus.setAcceleration(100.0);
  pinMode(Z_LIM, INPUT_PULLUP);

  //Stepper enable pin, high to diable drivers
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  // Joystick Jogging Enable
  pinMode(SW, INPUT_PULLUP); 

  if(!networkInit()) {
    Serial.println("Setup Failed.");
  }
  else {
    Serial.println("Setup Complete.");
  }
}

void loop() {

  static int indx=0;

if (indx == 0) {
  jogMirror();
}

/*
  if (tilt.distanceToGo() == 0) {
    tilt.moveTo(-tilt.currentPosition());
  }
  tilt.run();

  if(!(indx%10000)) {
    Serial.println(tilt.currentPosition());
  }
*/
  indx++;  
}
