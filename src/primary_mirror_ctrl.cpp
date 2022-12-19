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
@brief LFAST prototype Primary Mirror Control Interface function implementation
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror_ctrl.cpp

Implementation of Primary Mirror Control Functions.
*/

#include "primary_mirror_ctrl.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>
#include <cmath>
#include <cinttypes>
#include <iostream>
#include <TerminalInterface.h>
#include <math_util.h>
#include "device_config.h"
#include "teensy41_device.h"
#include "TimerOne.h"

#include <MultiStepper.h>
#include <AccelStepper.h>

AccelStepper Stepper_A(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper Stepper_B(AccelStepper::DRIVER, B_STEP, B_DIR);
AccelStepper Stepper_C(AccelStepper::DRIVER, C_STEP, C_DIR);
// MultiStepper steppers;
MultiStepper steppers;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Motion Control Functions  //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace LFAST;

void primaryMirrorControl_ISR()
{
    noInterrupts();
    PrimaryMirrorControl &pmc = PrimaryMirrorControl::getMirrorController();
    pmc.copyShadowToActive();
    pmc.pingMirrorControlStateMachine();
    interrupts();
}

PrimaryMirrorControl::PrimaryMirrorControl()
{
    controlMode = LFAST::PMC::STOP;
    currentMoveState = IDLE;
    hardware_setup();
}

PrimaryMirrorControl &PrimaryMirrorControl::getMirrorController()
{
    static PrimaryMirrorControl instance;
    return instance;
}

void PrimaryMirrorControl::hardware_setup()
{
    // Initialize motors + limit switches
    Stepper_A.setMaxSpeed(800.0);     // Steps per second
    Stepper_A.setAcceleration(100.0); // Steps per second per second
    pinMode(A_LIM, INPUT_PULLUP);

    Stepper_B.setMaxSpeed(800.0);
    Stepper_B.setAcceleration(100.0);
    pinMode(B_LIM, INPUT_PULLUP);

    Stepper_C.setMaxSpeed(800.0);
    Stepper_C.setAcceleration(100.0);
    pinMode(C_LIM, INPUT_PULLUP);

    steppers.addStepper(Stepper_A);
    steppers.addStepper(Stepper_B);
    steppers.addStepper(Stepper_C);
    // Global stepper enable pin, high to diable drivers
    pinMode(STEP_ENABLE_PIN, OUTPUT);

    // Initialize Timer
    Timer1.initialize(UPDATE_PRD_US);
    Timer1.stop();
    Timer1.attachInterrupt(primaryMirrorControl_ISR);

    // digitalWrite(STEP_ENABLE_PIN, DISABLE_STEPPER);
}

void PrimaryMirrorControl::setMoveNotifierFlag(volatile bool *flagPtr)
{
    moveNotifierFlagPtr = flagPtr;
}

void PrimaryMirrorControl::pingMirrorControlStateMachine()
{
    // tip/tilt/focus adustment control parsing
    bool moveCompleteFlag = false;
    static MOVE_STATE prevMoveState = IDLE;

    switch (currentMoveState)
    {
    case IDLE:
        if (checkForNewCommand())
        {
            currentMoveState = NEW_MOVE_CMD;
        }
        break;
    case NEW_MOVE_CMD:

        currentMoveState = MOVE_IN_PROGRESS;
        updateStepperCommands();
        // Intentional fall-through
    case MOVE_IN_PROGRESS:
        moveCompleteFlag = pingSteppers();
        saveCurrentPositionsToEeprom();
        if (moveCompleteFlag)
        {
            currentMoveState = MOVE_COMPLETE;
        }
        else
        {
            if (checkForNewCommand())
            {
                cli->printDebugMessage("Move interrupted.");
                currentMoveState = NEW_MOVE_CMD;
            }
        }
        static uint32_t counter = 0;
        // static uint32_t outerCounter = 0;
        if (counter++ >= 1000)
        {
            updateFeedbackFields();
            counter = 0;
            // cli->printfDebugMessage("Still here: %d", outerCounter++);
        }
        break;
    case MOVE_COMPLETE:
        updateFeedbackFields();
        // cli->printfDebugMessage("MOVE_COMPLETE");
        if (moveNotifierFlagPtr != nullptr)
            *moveNotifierFlagPtr = true;
        currentMoveState = IDLE;
        // Timer1.stop();
    }

    if (prevMoveState != currentMoveState)
    {
        updateStatusFields();
        prevMoveState = currentMoveState;
    }
    // cli->printfDebugMessage("Leaving ISR");
}

bool PrimaryMirrorControl::checkForNewCommand()
{
    bool result = false;
    if (controlMode == PMC::RELATIVE)
    {
        if (tipUpdated == true || tiltUpdated == true || focusUpdated == true)
        {
            result = true;
        }
    }
    else if (controlMode == PMC::ABSOLUTE)
    {
        if (tipUpdated == true && tiltUpdated == true && focusUpdated == true)
        {
            result = true;
        }
    }
    if (result)
    {
        tipUpdated = false;
        tiltUpdated = false;
        focusUpdated = false;
    }
    return result;
}

void PrimaryMirrorControl::enableControlInterrupt()
{
    Timer1.start();
    digitalWrite(STEP_ENABLE_PIN, ENABLE_STEPPER);
}

// Functions to update necessary control variables
void PrimaryMirrorControl::copyShadowToActive()
{
    CommandStates_Eng = ShadowCommandStates_Eng;
}

void PrimaryMirrorControl::setControlMode(uint8_t mode)
{
#if 0
    switch (controlMode)
    {
    case PMC::STOP:
        cli->printDebugMessage("STOP");
        break;
    case PMC::RELATIVE:
        cli->printDebugMessage("RELATIVE");
        break;
    case PMC::ABSOLUTE:
        cli->printDebugMessage("ABSOLUTE");
        break;
    }
#endif
    controlMode = mode;
}

void PrimaryMirrorControl::setTipTarget(double tgt)
{
    ShadowCommandStates_Eng.TIP_POS_ENG = tgt;
    tipUpdated = true;
    // cli->printfDebugMessage("TargetTip = %6.4f", CommandStates_Eng.TIP_POS_ENG);
}

void PrimaryMirrorControl::setTiltTarget(double tgt)
{
    ShadowCommandStates_Eng.TILT_POS_ENG = tgt;
    tiltUpdated = true;
    // cli->printfDebugMessage("TargetTilt = %6.4f", CommandStates_Eng.TILT_POS_ENG);
}

void PrimaryMirrorControl::setFocusTarget(double tgt)
{
    ShadowCommandStates_Eng.FOCUS_POS_ENG = tgt;
    focusUpdated = true;
    // cli->printfDebugMessage("TargetFocus = %6.4f", CommandStates_Eng.FOCUS_POS_ENG);
}

// Set the fan speed to a percentage S of full scale
// Fan Pin unknown?
void PrimaryMirrorControl::setFanSpeed(unsigned int PWR)
{
    analogWrite(FAN_CONTROL, PWR);
}

// Immediately stops all motion
void PrimaryMirrorControl::stopNow()
{
    digitalWrite(STEP_ENABLE_PIN, DISABLE_STEPPER);
    Stepper_A.stop();
    Stepper_B.stop();
    Stepper_C.stop();

    saveCurrentPositionsToEeprom();
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void PrimaryMirrorControl::updateStepperCommands()
{
    digitalWrite(STEP_ENABLE_PIN, ENABLE_STEPPER);
    // Convert Distance to steps (0.003mm per step??)
    A_cmdSteps = 0;
    B_cmdSteps = 0;
    C_cmdSteps = 0;
    CommandStates_Eng.getMotorPosnCommands(&A_cmdSteps, &B_cmdSteps, &C_cmdSteps);

    if (controlMode == PMC::RELATIVE)
    {
        cli->printfDebugMessage("Relative Step Commands: [A/B/C]: %d, %d, %d", A_cmdSteps, B_cmdSteps, C_cmdSteps);
        Stepper_A.move(A_cmdSteps);
        Stepper_B.move(B_cmdSteps);
        Stepper_C.move(C_cmdSteps);
    }
    else
    {
        cli->printfDebugMessage("Absolute Step Commands: [A/B/C]: %d, %d, %d", A_cmdSteps, B_cmdSteps, C_cmdSteps);
        long stepperCmdVector[3]{A_cmdSteps, B_cmdSteps, C_cmdSteps};
        steppers.moveTo(stepperCmdVector);
    }
    updateCommandFields();
}

bool PrimaryMirrorControl::pingSteppers()
{
    bool stepperADone = Stepper_A.currentPosition() == A_cmdSteps;
    bool stepperBDone = Stepper_B.currentPosition() == B_cmdSteps;
    bool stepperCDone = Stepper_C.currentPosition() == C_cmdSteps;
    bool moveCompleteFlag = stepperADone && stepperBDone && stepperCDone;
    if (!moveCompleteFlag)
    {
        steppers.run();
    }
    return moveCompleteFlag;
}
// Move all actuators to home positions at velocity V (steps/sec)
void PrimaryMirrorControl::goHome(volatile double homingSpeed)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    Stepper_A.setSpeed(-homingSpeed);
    Stepper_B.setSpeed(-homingSpeed);
    Stepper_C.setSpeed(-homingSpeed);
    // Retract motors until they reach limit switches
    while ((digitalRead(A_LIM)) || (digitalRead(B_LIM)) || (digitalRead(C_LIM)))
    {

        if ((digitalRead(A_LIM)))
        {
            Stepper_A.runSpeed();
        }
        if ((digitalRead(B_LIM)))
        {
            Stepper_B.runSpeed();
        }
        if ((digitalRead(C_LIM)))
        {
            Stepper_C.runSpeed();
        }
    }

    // Time to allow limit switch to settle
    delay(1);
    Stepper_A.setSpeed(homingSpeed * 0.2);
    Stepper_B.setSpeed(homingSpeed * 0.2);
    Stepper_C.setSpeed(homingSpeed * 0.2);
    // Deploy motors until limit switches deactivate
    while (!(digitalRead(A_LIM)) || !(digitalRead(B_LIM)) || !(digitalRead(C_LIM)))
    {

        if (!(digitalRead(A_LIM)))
        {
            Stepper_A.runSpeed();
        }
        if (!(digitalRead(B_LIM)))
        {
            Stepper_B.runSpeed();
        }
        if (!(digitalRead(C_LIM)))
        {
            Stepper_C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    Stepper_A.setCurrentPosition(0);
    Stepper_B.setCurrentPosition(0);
    Stepper_C.setCurrentPosition(0);
    saveCurrentPositionsToEeprom();
}

bool PrimaryMirrorControl::getStatus(uint8_t motor)
{
    if (motor == LFAST::PMC::MOTOR_A)
        return Stepper_A.isRunning(); // Checks to see if the motor is currently running to a target
    else if (motor == LFAST::PMC::MOTOR_B)
        return Stepper_B.isRunning(); // true if the speed is not zero or not at the target position
    else if (motor == LFAST::PMC::MOTOR_C)
        return Stepper_C.isRunning();
    else
        return false;
}

double PrimaryMirrorControl::getPosition(uint8_t motor)
{
    if (motor == LFAST::PMC::MOTOR_A)
        return Stepper_A.currentPosition(); // Checks to see if the motor is currently running to a target
    else if (motor == LFAST::PMC::MOTOR_B)
        return Stepper_B.currentPosition(); // true if the speed is not zero or not at the target position
    else if (motor == LFAST::PMC::MOTOR_C)
        return Stepper_C.currentPosition();
    else
        return 0.0;
}

void PrimaryMirrorControl::saveCurrentPositionsToEeprom()
{
    unsigned int eeAddr = 1;
    int Aposition = Stepper_A.currentPosition();
    int Bposition = Stepper_B.currentPosition();
    int Cposition = Stepper_C.currentPosition();
    // cli->printfDebugMessage("EEPROM Write [A/B/C]: %d, %d, %d", Aposition, Bposition, Cposition);
    EEPROM.put(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Cposition);
}

void PrimaryMirrorControl::resetPositionsInEeprom()
{
    unsigned int eeAddr = 1;
    EEPROM.put(eeAddr, 0);
    eeAddr += sizeof(int); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, 0);
    eeAddr += sizeof(int); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, 0);
    cli->printDebugMessage("Resetting eeprom positions", LFAST::WARNING);
}

void PrimaryMirrorControl::loadCurrentPositionsFromEeprom()
{
    unsigned int eeAddr = 1;
    int Aposition = 0;
    int Bposition = 0;
    int Cposition = 0;

    EEPROM.get(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); // Move address to the next byte after float 'f'.
    EEPROM.get(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); // Move address to the next byte after float 'f'.
    EEPROM.get(eeAddr, Cposition);

    cli->printfDebugMessage("EEPROM Load [A/B/C]: %d, %d, %d", Aposition, Bposition, Cposition);
    Stepper_A.setCurrentPosition(Aposition);
    Stepper_B.setCurrentPosition(Bposition);
    Stepper_C.setCurrentPosition(Cposition);
}

void PrimaryMirrorControl::setupPersistentFields()
{
    // None to set up yet
    // TEST_SERIAL.printf("\r\n%s[setupPersistentFields]: %x\r\n", cli);
    if (cli == nullptr)
        return;

    cli->addPersistentField(this->DeviceName, "[CMD MODE]", CMD_MODE_ROW);
    cli->addPersistentField(this->DeviceName, "[TIP CMD]", TIP_ROW);
    cli->addPersistentField(this->DeviceName, "[TILT CMD]", TILT_ROW);
    cli->addPersistentField(this->DeviceName, "[FOCUS CMD]", FOCUS_ROW);
    cli->addPersistentField(this->DeviceName, "[STATE]", MOVE_SM_STATE_ROW);
    cli->addPersistentField(this->DeviceName, "[STEPPER A]", STEPPER_A_FB);
    cli->addPersistentField(this->DeviceName, "[STEPPER B]", STEPPER_B_FB);
    cli->addPersistentField(this->DeviceName, "[STEPPER C]", STEPPER_C_FB);
    updateStatusFields();
}

void PrimaryMirrorControl::updateStatusFields()
{
    switch (currentMoveState)
    {
    case IDLE:
        cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "IDLE");
        break;
    case NEW_MOVE_CMD:
        cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "NEW_MOVE_CMD");
        break;
    case MOVE_IN_PROGRESS:
        cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "MOVE_IN_PROGRESS");
        break;
    case MOVE_COMPLETE:
        cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "MOVE_COMPLETE");
        break;
    }

    switch (controlMode)
    {
    case PMC::STOP:
        cli->updatePersistentField(DeviceName, CMD_MODE_ROW, "STOP");
        break;
    case PMC::RELATIVE:
        cli->updatePersistentField(DeviceName, CMD_MODE_ROW, "RELATIVE");
        break;
    case PMC::ABSOLUTE:
        cli->updatePersistentField(DeviceName, CMD_MODE_ROW, "ABSOLUTE");
        break;
    }
}

void PrimaryMirrorControl::updateCommandFields()
{
    cli->updatePersistentField(DeviceName, TIP_ROW, CommandStates_Eng.TIP_POS_ENG);
    cli->updatePersistentField(DeviceName, TILT_ROW, CommandStates_Eng.TILT_POS_ENG);
    cli->updatePersistentField(DeviceName, FOCUS_ROW, CommandStates_Eng.FOCUS_POS_ENG);
}

void PrimaryMirrorControl::updateFeedbackFields()
{
    cli->updatePersistentField(DeviceName, STEPPER_A_FB, Stepper_A.currentPosition());
    cli->updatePersistentField(DeviceName, STEPPER_B_FB, Stepper_B.currentPosition());
    cli->updatePersistentField(DeviceName, STEPPER_C_FB, Stepper_C.currentPosition());
}