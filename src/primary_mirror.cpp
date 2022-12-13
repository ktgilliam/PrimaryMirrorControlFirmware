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
#include <TcpCommsService.h>
#include <TerminalInterface.h>
#include <teensy41_device.h>
// Parsing of JSON style command done in network file, for now.

/*
Features:
Manual Control: Joystick, Arrow Keys

To Do:
What are desired max speeds of Stepper Motors?


Threading
  - 'TeensyThreads.h'
  - Activate function as thread to allow other function to take over if needed

System Recovery
  - Remember positions of stepepr motors in case of ungraceful shutdown. (EEPROM)

fanSpeed Control function
*/

void handshake(unsigned int val);

LFAST::TcpCommsService *commsService;
TerminalInterface *pmcIf;

byte myIP[] IPAdd;
unsigned int mPort = PORT;

void control_thread()
{
  commsService->processClientData("PMCMessage");
  threads.yield();
}

void comm_thread()
{

  while (1)
  {
    if (commsService->checkForNewClients())
    {
      if (commsService->checkForNewClientData())
      {
        pmcIf->addDebugMessage("New data received.");
        set_thread_ID(0, threads.addThread(control_thread));
      }
      commsService->stopDisconnectedClients();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  hardware_setup();

  // Joystick Jogging Enable: INW
  // pinMode(SW, INPUT_PULLUP);

  // Fan PWM Enable

  commsService = new LFAST::TcpCommsService(myIP);
  commsService->initializeEnetIface(PORT); // initialize
  pmcIf = new TerminalInterface(PMC_LABEL, &(TEST_SERIAL), TEST_SERIAL_BAUD);
  commsService->connectTerminalInterface(pmcIf);
  copyTerminalInterfacePtr(pmcIf);
  copyCommsServicePtr(commsService);

  if (!commsService->Status())
  {
    pmcIf->addDebugMessage("Device Setup Failed.");
    while (true)
    {
      ;
      ;
    }
  }

  commsService->registerMessageHandler<unsigned int>("Handshake", handshake);
  commsService->registerMessageHandler<double>("FindHome", home);
  commsService->registerMessageHandler<unsigned int>("MoveType", moveType);
  commsService->registerMessageHandler<double>("SetVelocity", changeVel);
  commsService->registerMessageHandler<unsigned int>("VelUnits", velUnits);
  commsService->registerMessageHandler<double>("SetTip", changeTip);
  commsService->registerMessageHandler<double>("SetTilt", changeTilt);
  commsService->registerMessageHandler<double>("SetFocus", changeFocus);
  commsService->registerMessageHandler<double>("GetStatus", getStatus);
  commsService->registerMessageHandler<double>("GetPositions", getPositions);
  commsService->registerMessageHandler<double>("Jog", jogMirror);
  commsService->registerMessageHandler<double>("Stop", stop);
  commsService->registerMessageHandler<unsigned int>("SetFanSpeed", fanSpeed);

  delay(500);

  load_current_positions();

  threads.setDefaultStackSize(6000);
  set_thread_ID(threads.addThread(comm_thread), 0);


  pmcIf->addDebugMessage("Initialization complete");
}

void loop()
{

  if (!commsService->Status())
  {
    pmcIf->addDebugMessage("Reconnecting to network.");
    commsService->initializeEnetIface(PORT); // initialize
    while (true)
    {
      ;
      ;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////