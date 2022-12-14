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
#include "primary_mirror_global.h"
#include "teensy41_device.h"

#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper Stepper_A(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper Stepper_B(AccelStepper::DRIVER, B_STEP, B_DIR);
AccelStepper Stepper_C(AccelStepper::DRIVER, C_STEP, C_DIR);



//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Motion Control Functions  //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace LFAST;

PrimaryMirrorControl::PrimaryMirrorControl()
{
    controlMode = LFAST::PMC::STOP;
    loadCurrentPositionsFromEeprom();
    hardware_setup();
}

PrimaryMirrorControl &PrimaryMirrorControl::getMirrorController()
{
    static PrimaryMirrorControl instance;
    return instance;
}

void PrimaryMirrorControl::hardware_setup()
{
    stepperControl = new MultiStepper();

    // Initialize motors + limit switches
    Stepper_A.setMaxSpeed(800.0);     // Steps per second
    Stepper_A.setAcceleration(100.0); // Steps per second per second
    stepperControl->addStepper(Stepper_A);
    pinMode(A_LIM, INPUT_PULLUP);

    Stepper_B.setMaxSpeed(800.0);
    Stepper_B.setAcceleration(100.0);
    stepperControl->addStepper(Stepper_B);
    pinMode(B_LIM, INPUT_PULLUP);

    Stepper_C.setMaxSpeed(800.0);
    Stepper_C.setAcceleration(100.0);
    stepperControl->addStepper(Stepper_C);
    pinMode(C_LIM, INPUT_PULLUP);

    // Global stepper enable pin, high to diable drivers
    pinMode(STEP_ENABLE_PIN, OUTPUT);
    digitalWrite(STEP_ENABLE_PIN, HIGH);

}

void PrimaryMirrorControl::setupPersistentFields()
{
    // None to set up yet
    // TEST_SERIAL.printf("\r\n%s[setupPersistentFields]: %x\r\n", cli);
    if (cli == nullptr)
        return;

    cli->addPersistentField(this->DeviceName, "[TIP]", TIP_ROW);
    cli->addPersistentField(this->DeviceName, "[TILT]", TILT_ROW);
    cli->addPersistentField(this->DeviceName, "[FOCUS]", FOCUS_ROW);
}
// Functions to update necessary control variables

void PrimaryMirrorControl::moveMirror()
{
    // tip/tilt adustment control parsing
    if (tipUpdated == true || tiltUpdated == true || focusUpdated == true)
    {
        moveSteppers();
        cli->printfDebugMessage("[Tip/Tilt/Focus] = %6.4f, %6.4f, %6.4f", CommandStates_Eng.TIP_POS_ENG, CommandStates_Eng.TILT_POS_ENG, CommandStates_Eng.FOCUS_POS_ENG);
        cli->updatePersistentField(DeviceName, TIP_ROW, CommandStates_Eng.TIP_POS_ENG);
        cli->updatePersistentField(DeviceName, TILT_ROW, CommandStates_Eng.TILT_POS_ENG);
        cli->updatePersistentField(DeviceName, FOCUS_ROW, CommandStates_Eng.FOCUS_POS_ENG);
    }
}

void PrimaryMirrorControl::setControlMode(uint8_t mode)
{
    controlMode = mode;
}

void PrimaryMirrorControl::setTipTarget(double tgt)
{
    CommandStates_Eng.TIP_POS_ENG = tgt;
    tipUpdated = true;
    cli->printfDebugMessage("TargetTip = %6.4f", CommandStates_Eng.TIP_POS_ENG);
}

void PrimaryMirrorControl::setTiltTarget(double tgt)
{
    CommandStates_Eng.TILT_POS_ENG = tgt;
    tiltUpdated = true;
    cli->printfDebugMessage("TargetTilt = %6.4f", CommandStates_Eng.TILT_POS_ENG);
}

void PrimaryMirrorControl::setFocusTarget(double tgt)
{
    CommandStates_Eng.FOCUS_POS_ENG = tgt;
    focusUpdated = true;
    cli->printfDebugMessage("TargetFocus = %6.4f", CommandStates_Eng.FOCUS_POS_ENG);
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
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    Stepper_A.stop();
    Stepper_B.stop();
    Stepper_C.stop();

    saveCurrentPositionsToEeprom();
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void PrimaryMirrorControl::moveSteppers()
{
    if(tipUpdated && tiltUpdated && focusUpdated)
    {
    digitalWrite(STEP_ENABLE_PIN, ENABLE_STEPPER);
    // Convert Distance to steps (0.003mm per step??)
    int32_t A_cmdSteps = 0;
    int32_t B_cmdSteps = 0;
    int32_t C_cmdSteps = 0;

    CommandStates_Eng.getMotorPosnCommands(&A_cmdSteps, &B_cmdSteps, &C_cmdSteps);

    if (controlMode == PMC::RELATIVE)
    {
        A_cmdSteps += Stepper_A.currentPosition();
        B_cmdSteps += Stepper_B.currentPosition();
        C_cmdSteps += Stepper_C.currentPosition();
    }

    long stepperCmdVector[3]{A_cmdSteps, B_cmdSteps, C_cmdSteps};
    stepperControl->moveTo(stepperCmdVector);
    digitalWrite(STEP_ENABLE_PIN, DISABLE_STEPPER);
    saveCurrentPositionsToEeprom();
    }
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
    Stepper_A.setSpeed(homingSpeed*0.2);
    Stepper_B.setSpeed(homingSpeed*0.2);
    Stepper_C.setSpeed(homingSpeed*0.2);
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

    EEPROM.put(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Cposition);
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

    Stepper_A.setCurrentPosition(Aposition);
    Stepper_B.setCurrentPosition(Bposition);
    Stepper_C.setCurrentPosition(Cposition);
}
