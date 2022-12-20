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

#include <Arduino.h>
// #include <TeensyThreads.h>
#include "Watchdog_t4.h"

#include <cmath>
#include <math.h>
#include <string>

#include <TcpCommsService.h>
#include <TerminalInterface.h>
#include <teensy41_device.h>

#include "device_config.h"
#include "primary_mirror_ctrl.h"
// Parsing of JSON style command done in network file, for now.
#include "CrashReport.h"

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
void moveType(unsigned int type);
void velUnits(unsigned int targetUnits);
void home(double v);
void changeVel(double targetVel);
void changeTip(double targetTip);
void changeTilt(double targetTilt);
void changeFocus(double targetFocus);

void getStatus(double lst);
void getPositions(double lst);
void stop(double lst);
void fanSpeed(unsigned int val);

LFAST::TcpCommsService *commsService;
PrimaryMirrorControl *pPmc;
TerminalInterface *cli;

byte myIP[] IPAdd;
unsigned int mPort = PORT;

volatile bool moveCompleteFlag = false;
volatile bool homingCompleteFlag = false;

WDT_T4<WDT1> wdt;
bool wdt_ready = false;
void watchdogWarning()
{
  if (cli != nullptr)
  {
    cli->printDebugMessage("Danger - feed the dog!", LFAST::WARNING);
  }
}
void configureWatchdog()
{
  if (cli != nullptr)
  {
    cli->printDebugMessage("Starting watchdog");
  }
  WDT_timings_t config;
  config.trigger = 5;  /* in seconds, 0->128 */
  config.timeout = 10; /* in seconds, 0->128 */
  config.callback = watchdogWarning;
  config.pin = 13;
  pinMode(13, OUTPUT);
  wdt_ready = true;
  wdt.begin(config);
}

void setup()
{
  PrimaryMirrorControl &pmc = PrimaryMirrorControl::getMirrorController();
  pPmc = &pmc;
  commsService = new LFAST::TcpCommsService(myIP);
  cli = new TerminalInterface(PMC_LABEL, &(TEST_SERIAL), TEST_SERIAL_BAUD);
  commsService->connectTerminalInterface(cli, "Comms");
  pPmc->connectTerminalInterface(cli, "pmc");
  cli->printPersistentFieldLabels();

  commsService->initializeEnetIface(PORT); // initialize

  if (!commsService->Status())
  {
    cli->printDebugMessage("Device Setup Failed.");
    while (true)
    {
      ;
    }
  }

  commsService->registerMessageHandler<unsigned int>("Handshake", handshake);
  commsService->registerMessageHandler<unsigned int>("MoveType", moveType);
  commsService->registerMessageHandler<double>("FindHome", home);
  commsService->registerMessageHandler<double>("SetTip", changeTip);
  commsService->registerMessageHandler<double>("SetTilt", changeTilt);
  commsService->registerMessageHandler<double>("SetFocus", changeFocus);
  commsService->registerMessageHandler<double>("GetStatus", getStatus);
  commsService->registerMessageHandler<double>("GetPositions", getPositions);
  commsService->registerMessageHandler<double>("Stop", stop);
  commsService->registerMessageHandler<unsigned int>("SetFanSpeed", fanSpeed);

  delay(500);
  pPmc->resetPositionsInEeprom();

  pPmc->setMoveNotifierFlag(&moveCompleteFlag);
  pPmc->setHomingCompleteNotifierFlag(&homingCompleteFlag);

  pPmc->loadCurrentPositionsFromEeprom();
  cli->printDebugMessage("Initialization complete");
  // cli->printDebugMessage(DEBUG_CODE_ID_STR);

  if (CrashReport)
  {
    CrashReport.printTo(TEST_SERIAL);
    TEST_SERIAL.print("\nPower cycle to clear this error.");
    while (1)
    {
      ;
    }
  }
}

void loop()
{
  if (wdt_ready)
    wdt.feed();
  commsService->checkForNewClients();
  if (commsService->checkForNewClientData())
  {
    // cli->printDebugMessage("New data received.");
    commsService->processClientData("PMCMessage");
  }
  commsService->stopDisconnectedClients();
  delayMicroseconds(1000);

  if (moveCompleteFlag)
  {
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<bool>("MoveComplete", true);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
    cli->printDebugMessage("Move Complete.");
    moveCompleteFlag = false;
  }
  if(homingCompleteFlag)
  {
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<bool>("HomingComplete", true);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
    cli->printDebugMessage("Homing Complete.");
    homingCompleteFlag = false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handshake function to confirm connection
void handshake(unsigned int val)
{
  if (val == 0xDEAD)
  {
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<unsigned int>("Handshake", 0xBEEF);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
    cli->printDebugMessage("Connected to client, starting control ISR.");
    if (!wdt_ready)
      configureWatchdog();
    pPmc->enableControlInterrupt();
  }
  return;
}

void moveType(unsigned int type)
{
  pPmc->setControlMode(type);
}

void home(double v)
{
  pPmc->goHome(v);
  LFAST::CommsMessage newMsg;
  newMsg.addKeyValuePair<std::string>("Finding Home", "$OK^");
  commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

void changeTip(double targetTip)
{
  // noInterrupts();
  pPmc->setTipTarget(targetTip);
  // interrupts();

  // moveMirror(LFAST::TIP, targetTip);
}

void changeTilt(double targetTilt)
{
  // noInterrupts();
  pPmc->setTiltTarget(targetTilt);
  // interrupts();

  // moveMirror(LFAST::TILT, targetTilt);
}

void changeFocus(double targetFocus)
{

  // noInterrupts();
  pPmc->setFocusTarget(targetFocus);
  // interrupts();
}

void fanSpeed(unsigned int PWR)
{
  // noInterrupts();
  pPmc->setFanSpeed(PWR);
  // interrupts();
}

// Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving
void getStatus(double lst)
{
  LFAST::CommsMessage newMsg;
  newMsg.addKeyValuePair<bool>("ARunning?", pPmc->getStatus(LFAST::PMC::MOTOR_A));
  newMsg.addKeyValuePair<bool>("BRunning?", pPmc->getStatus(LFAST::PMC::MOTOR_B));
  newMsg.addKeyValuePair<bool>("CRunning?", pPmc->getStatus(LFAST::PMC::MOTOR_C));
  commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

void stop(double lst)
{
  pPmc->stopNow();
  // int i = 1;
  // while ((ctrlthreadID - i) != commthreadID) // Kill all running control threads
  // {
  //   threads.kill(ctrlthreadID - i);
  //   i++;
  // }
  LFAST::CommsMessage newMsg;
  newMsg.addKeyValuePair<std::string>("Stopped", "$OK^");
  commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

// Returns 3 step counts
void getPositions(double lst)
{
  LFAST::CommsMessage newMsg;
  newMsg.addKeyValuePair<double>("APosition", pPmc->getStepperPosition(LFAST::PMC::MOTOR_A));
  newMsg.addKeyValuePair<double>("BPosition", pPmc->getStepperPosition(LFAST::PMC::MOTOR_B));
  newMsg.addKeyValuePair<double>("CPosition", pPmc->getStepperPosition(LFAST::PMC::MOTOR_C));
  commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////