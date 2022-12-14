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
#include <TimerOne.h>
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
AccelStepper Stpper_C(AccelStepper::DRIVER, C_STEP, C_DIR);

MultiStepper stepperControl();

const unsigned long int updatePrd_us = 100;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Motion Control Functions  //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace LFAST;

void LFAST::enableControlLoopInterrupt()
{
    Timer1.attachInterrupt(LFAST::updateControlLoop_ISR);
    Timer1.initialize(updatePrd_us);
}

void LFAST::updateControlLoop_ISR()
{
    LFAST::PrimaryMirrorControl &pmc = LFAST::PrimaryMirrorControl::getMirrorController();
    // pmc.cli->printDebugMessage("inside interrupt");
    pmc.moveMirror();
}

PrimaryMirrorControl::PrimaryMirrorControl()
{
    // PrimaryMirrorControl::pmcPtr = this;
    controlMode = LFAST::PMC::STOP;
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

    Stpper_C.setMaxSpeed(800.0);
    Stpper_C.setAcceleration(100.0);
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
        if (unitVal == LFAST::PMC::ENGINEERING)
        {
            if (controlMode == LFAST::PMC::ABSOLUTE)
            {
                cli->printDebugMessage("moveAbsolute");
                moveMirror();
            }
            else if (controlMode == LFAST::PMC::RELATIVE)
            {
                cli->printDebugMessage("moveRelative");
                moveRelative();
            }
        }
        tipUpdated = false;
        tiltUpdated = false;
    }
}

void PrimaryMirrorControl::setVelocity(double vel)
{
    CommandStates_Eng.MOVE_SPEED_ENG = vel;
}

void PrimaryMirrorControl::setControlMode(uint8_t mode)
{
    controlMode = mode;
}

void PrimaryMirrorControl::setVelUnits(uint8_t velUnits)
{
    unitVal = velUnits;
}

void PrimaryMirrorControl::setTipTarget(double tgt)
{
    CommandStates_Eng.TIP_POS_ENG = tgt;
    tipUpdated = true;
}

void PrimaryMirrorControl::setTiltTarget(double tgt)
{
    CommandStates_Eng.TILT_POS_ENG = tgt;
    tiltUpdated = true;
}

void PrimaryMirrorControl::setFocusTarget(double tgt)
{
    CommandStates_Eng.FOCUS_POS_ENG = tgt;
    focusUpdated = true;
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
    Stpper_C.stop();

    save_current_positions();
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void PrimaryMirrorControl::moveMirror()
{
    digitalWrite(STEP_ENABLE_PIN, LOW);

    // Convert Distance to steps (0.003mm per step??)
    int32_t A_cmdSteps = 0;
    int32_t B_cmdSteps = 0;
    int32_t C_cmdSteps = 0;
    int32_t v_cmdStepsPerSec = 0;
    CommandStates_Eng.getMotorPosnCommands(&A_cmdSteps, &B_cmdSteps, &C_cmdSteps);
    CommandStates_Eng.getMotorSpeedCommand(&v_cmdStepsPerSec);

    if (controlMode == PMC::ABSOLUTE)
    {
        float A_errorSteps = (float)A_cmdSteps - Stepper_A.currentPosition();
        float V_cmd_A = v_cmdStepsPerSec * sign(A_errorSteps);
        if (std::abs(A_errorSteps) > 0)
        {
            Stepper_A.setSpeed(V_cmd_A);
            Stepper_A.runSpeed();
        }

        float B_errorSteps = (float)B_cmdSteps - Stepper_B.currentPosition();
        float V_cmd_B = v_cmdStepsPerSec * sign(B_errorSteps);
        if (std::abs(B_errorSteps) > 0)
        {
            Stepper_B.setSpeed(V_cmd_B);
            Stepper_B.runSpeed();
        }


        float C_errorSteps = (float)C_cmdSteps - Stpper_C.currentPosition();
        float V_cmd_C = v_cmdStepsPerSec * sign(C_errorSteps);
        if (std::abs(A_errorSteps) > 0)
        {
            Stpper_C.setSpeed(V_cmd_C);
            Stpper_C.runSpeed();
        }
    }
    else if (controlMode == PMC::RELATIVE)
    {
        Stepper_A.moveTo(Stepper_A.currentPosition() + A_cmdSteps);
        Stepper_B.moveTo(Stepper_B.currentPosition() + B_cmdSteps);
        Stpper_C.moveTo(Stpper_C.currentPosition() + C_cmdSteps);
    }
    else
    {
    }

    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

// v input in (steps/sec), z input as um
void PrimaryMirrorControl::focusRawAbsolute(double v, double z)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    // LFAST::CommsMessage newMsg;
    // newMsg.addKeyValuePair<std::string>("Adjusting focus absolute.", "$OK^");
    // commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // get z position of actuators in steps (x, y positions are fixed)
    int zA, zB, zC, zf;
    zA = Stepper_A.currentPosition();
    zB = Stepper_B.currentPosition();
    zC = Stpper_C.currentPosition();

    // convert position in steps to mm?
    // MM_PER_STEP microns / step
    zA = (zA * MM_PER_STEP);
    zB = (zB * MM_PER_STEP);
    zC = (zC * MM_PER_STEP);

    // Calculate z postion of focus given three actuator points.
    // Derived from the position of actuators at any givem moment
    // Equation produces position in mm
    zf = ((((1376420) * (zB - (2 * zA) + zC)) / (4129260)) + zA) * 1000; // *1000 to convert mm to um
    // Adjust velocity ( + for up z movement, - for down z movement)
    if (z < zf)
    {
        v = -abs(v);
    }
    else
    {
        v = abs(v);
    }

    // Difference between desired position and actual position to determine movement amount
    // cli->printDebugMessage(z);
    int move = z - zf;
    // convert micron movement back to steps
    move = move / MICRON_PER_STEP; // microns * (1 step / 3 microns)
    // cli->printDebugMessage(move);

    // Equally adust desired actuator movement given desired focus position
    Stepper_A.setSpeed(v);
    Stepper_A.moveTo(Stepper_A.currentPosition() + move);
    Stepper_B.setSpeed(v);
    Stepper_B.moveTo(Stepper_B.currentPosition() + move);
    Stpper_C.setSpeed(v);
    Stpper_C.moveTo(Stpper_C.currentPosition() + move);

    while ((Stepper_A.distanceToGo() != 0) || (Stepper_B.distanceToGo() != 0) || (Stpper_C.distanceToGo() != 0))
    {

        if (Stepper_A.distanceToGo() != 0)
        {
            Stepper_A.runSpeed();
        }
        if (Stepper_B.distanceToGo() != 0)
        {
            Stepper_B.runSpeed();
        }
        if (Stpper_C.distanceToGo() != 0)
        {
            Stpper_C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

// Move all actuators to home positions at velocity V (steps/sec)
void PrimaryMirrorControl::goHome(volatile double v)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    Stepper_A.setSpeed(-v);
    Stepper_B.setSpeed(-v);
    Stpper_C.setSpeed(-v);
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
            Stpper_C.runSpeed();
        }
    }

    // Time to allow limit switch to settle
    delay(1);
    Stepper_A.setSpeed(v);
    Stepper_B.setSpeed(v);
    Stpper_C.setSpeed(v);
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
            Stpper_C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    Stepper_A.setCurrentPosition(0);
    Stepper_B.setCurrentPosition(0);
    Stpper_C.setCurrentPosition(0);
    save_current_positions();
}

bool PrimaryMirrorControl::getStatus(uint8_t motor)
{
    if (motor == LFAST::PMC::MOTOR_A)
        return Stepper_A.isRunning(); // Checks to see if the motor is currently running to a target
    else if (motor == LFAST::PMC::MOTOR_B)
        return Stepper_B.isRunning(); // true if the speed is not zero or not at the target position
    else if (motor == LFAST::PMC::MOTOR_C)
        return Stpper_C.isRunning();
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
        return Stpper_C.currentPosition();
    else
        return 0.0;
}

void PrimaryMirrorControl::save_current_positions()
{
    unsigned int eeAddr = 1;
    int Aposition = Stepper_A.currentPosition();
    int Bposition = Stepper_B.currentPosition();
    int Cposition = Stpper_C.currentPosition();

    EEPROM.put(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); // Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Cposition);
}

void PrimaryMirrorControl::load_current_positions()
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
    Stpper_C.setCurrentPosition(Cposition);
}
