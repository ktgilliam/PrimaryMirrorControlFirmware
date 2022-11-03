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
//#include "primary_mirror_network.h"



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

fanSpeed Control function 
*/

// Figure out appropriate pin outs
//AccelStepper name(mode, step_pin, direction_pin)
AccelStepper A(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper B(AccelStepper::DRIVER, B_STEP, B_DIR);
AccelStepper C(AccelStepper::DRIVER, C_STEP, C_DIR);

LFAST::TcpCommsService *commsService;

byte myIP[] IPAdd;
unsigned int mPort = PORT;


void setup() {
  Serial.begin(9600);

  // Initialize motors + limit switches
  A.setMaxSpeed(800.0); // Steps per second
  A.setAcceleration(100.0); // Steps per second per second
  pinMode(A_LIM, INPUT_PULLUP);

  B.setMaxSpeed(800.0);
  B.setAcceleration(100.0);
  pinMode(B_LIM, INPUT_PULLUP);

  C.setMaxSpeed(800.0);
  C.setAcceleration(100.0);
  pinMode(C_LIM, INPUT_PULLUP);

  //Global stepper enable pin, high to diable drivers
  pinMode(STEP_ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_ENABLE_PIN, HIGH);

  // Joystick Jogging Enable
  pinMode(SW, INPUT_PULLUP); 

  // Fan PWM Enable


  /*
  commsService = new LFAST::EthernetCommsService(myIP, mPort);
  if (!commsService->Status()) {
        TEST_SERIAL.println("Device Setup Failed.");
        while (true)
        {
            ;
            ;
        }
    }

  commsService->registerMessageHandler<double>("FindHome", home);
  commsService->registerMessageHandler<double>("SetVelocity", changeVel);
  commsService->registerMessageHandler<double>("SetTip", changeTip);
  commsService->registerMessageHandler<double>("SetTilt", changeTilt);
  commsService->registerMessageHandler<double>("GetStatus", getStatus);
  commsService->registerMessageHandler<double>("GetPositions", getPositions);
  commsService->registerMessageHandler<double>("Jog", jogMirror);
  commsService->registerMessageHandler<double>("Stop", stop);
  commsService->registerMessageHandler<unsigned int>("SetFanSpeed", fanSpeed);
  */
}


void loop() {

  static int indx = 0;

  /*
  commsService->checkForNewClients();
  commsService->checkForNewClientData();
  commsService->processClientData();
  commsService->stopDisconnectedClients();
  */

  if (!(indx%1000)) {
    Serial.println("Executing.");
    home(100);
    //moveRawAbsolute(200, 1, 0);
  }
  
  indx++;  
}


