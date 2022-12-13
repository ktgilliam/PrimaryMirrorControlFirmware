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
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <cstring>
#include <TimerOne.h>

#include "primary_mirror_global.h"

#include <iostream>
#include <TerminalInterface.h>

#include "teensy41_device.h"

AccelStepper A(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper B(AccelStepper::DRIVER, B_STEP, B_DIR);
AccelStepper C(AccelStepper::DRIVER, C_STEP, C_DIR);

const unsigned long int updatePrd_us = 1000;
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
    // pmc.cli->addDebugMessage("inside interrupt");
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
    A.setMaxSpeed(800.0);     // Steps per second
    A.setAcceleration(100.0); // Steps per second per second
    pinMode(A_LIM, INPUT_PULLUP);

    B.setMaxSpeed(800.0);
    B.setAcceleration(100.0);
    pinMode(B_LIM, INPUT_PULLUP);

    C.setMaxSpeed(800.0);
    C.setAcceleration(100.0);
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
    // if (cli != nullptr)
    // {
    //     cli->addDebugMessage("moving mirror");
    // }
    // else
    // {
    //     TEST_SERIAL.println("no cli");
    // }
    // tip/tilt adustment control parsing
    if (tipUpdated == true || tiltUpdated == true)
    {
        if ((controlMode == LFAST::PMC::ABSOLUTE) && (unitVal == LFAST::PMC::ENGINEERING))
        {
            cli->addDebugMessage("moveAbsolute");
            moveAbsolute(velVal, tipVal, tiltVal);
        }
        else if ((controlMode == LFAST::PMC::RELATIVE) && (unitVal == LFAST::PMC::ENGINEERING))
        {
            cli->addDebugMessage("moveRelative");
            moveRelative(velVal, tipVal, tiltVal);
        }
        else if ((controlMode == LFAST::PMC::ABSOLUTE) && (unitVal == LFAST::PMC::STEPS_PER_SEC))
        {
            cli->addDebugMessage("moveRawAbsolute");
            moveRawAbsolute(velVal, tipVal, tiltVal);
        }
        else if ((controlMode == LFAST::PMC::RELATIVE) && (unitVal == LFAST::PMC::STEPS_PER_SEC))
        {
            cli->addDebugMessage("moveRawRelative");
            moveRawRelative(velVal, tipVal, tiltVal);
        }
        tipUpdated = false;
        tiltUpdated = false;
    }
    // focus adjustment control parsing
    // else if (focusUpdated == true && modeUpdated == true && velUpdated == true && unitUpdated == true)
    if (focusUpdated == true)
    {
        if (controlMode == LFAST::PMC::RELATIVE && unitVal == LFAST::PMC::ENGINEERING)
        {
            cli->addDebugMessage("focusRelative");
            focusRelative(velVal, focusVal);
        }
        else if (controlMode == LFAST::PMC::RELATIVE && unitVal == LFAST::PMC::STEPS_PER_SEC)
        {
            cli->addDebugMessage("focusRelativeRaw");
            focusRelativeRaw(velVal, focusVal);
        }
        else if (controlMode == LFAST::PMC::ABSOLUTE && unitVal == LFAST::PMC::ENGINEERING)
        {
            cli->addDebugMessage("focusAbsolute");
            focusAbsolute(velVal, focusVal);
        }
        else if (controlMode == LFAST::PMC::ABSOLUTE && unitVal == LFAST::PMC::ENGINEERING)
        {
            cli->addDebugMessage("focusRawAbsolute");
            focusRawAbsolute(velVal, focusVal);
        }
        focusUpdated = false;
    }
}

void PrimaryMirrorControl::setVelocity(double vel)
{
    velVal = vel;
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
    tipVal = tgt;
    tipUpdated = true;
}

void PrimaryMirrorControl::setTiltTarget(double tgt)
{
    tiltVal = tgt;
    tiltUpdated = true;
}

void PrimaryMirrorControl::setFocusTarget(double tgt)
{
    focusVal = tgt;
    focusUpdated = true;
}
// Set the fan speed to a percentage S of full scale
// Fan Pin unknown?
void PrimaryMirrorControl::setFanSpeed(unsigned int PWR)
{
    analogWrite(FAN_CONTROL, PWR);
}

bool PrimaryMirrorControl::getStatus(uint8_t motor)
{
    if (motor == LFAST::PMC::MOTOR_A)
        return A.isRunning(); // Checks to see if the motor is currently running to a target
    else if (motor == LFAST::PMC::MOTOR_B)
        return B.isRunning(); // true if the speed is not zero or not at the target position
    else if (motor == LFAST::PMC::MOTOR_C)
        return C.isRunning();
    else
        return false;
}

double PrimaryMirrorControl::getPosition(uint8_t motor)
{
    if (motor == LFAST::PMC::MOTOR_A)
        return A.currentPosition(); // Checks to see if the motor is currently running to a target
    else if (motor == LFAST::PMC::MOTOR_B)
        return B.currentPosition(); // true if the speed is not zero or not at the target position
    else if (motor == LFAST::PMC::MOTOR_C)
        return C.currentPosition();
    else
        return 0.0;
}

// Immediately stops all motion
void PrimaryMirrorControl::stopNow()
{
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    A.stop();
    B.stop();
    C.stop();

    save_current_positions();
}

// Move all actuators to home positions at velocity V (steps/sec)
void PrimaryMirrorControl::goHome(volatile double v)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    A.setSpeed(-v);
    B.setSpeed(-v);
    C.setSpeed(-v);
    // Retract motors until they reach limit switches
    while ((digitalRead(A_LIM)) || (digitalRead(B_LIM)) || (digitalRead(C_LIM)))
    {

        if ((digitalRead(A_LIM)))
        {
            A.runSpeed();
        }
        if ((digitalRead(B_LIM)))
        {
            B.runSpeed();
        }
        if ((digitalRead(C_LIM)))
        {
            C.runSpeed();
        }
    }

    // Time to allow limit switch to settle
    delay(1);
    A.setSpeed(v);
    B.setSpeed(v);
    C.setSpeed(v);
    // Deploy motors until limit switches deactivate
    while (!(digitalRead(A_LIM)) || !(digitalRead(B_LIM)) || !(digitalRead(C_LIM)))
    {

        if (!(digitalRead(A_LIM)))
        {
            A.runSpeed();
        }
        if (!(digitalRead(B_LIM)))
        {
            B.runSpeed();
        }
        if (!(digitalRead(C_LIM)))
        {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    A.setCurrentPosition(0);
    B.setCurrentPosition(0);
    C.setCurrentPosition(0);
    save_current_positions();
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// Velocity input as rad/ sec, converted to steps / sec
void PrimaryMirrorControl::moveAbsolute(double v, double tip, double tilt)
{
    // Convert v (rad / second) to steps / second
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel
    moveRawAbsolute(v, tip, tilt);
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void PrimaryMirrorControl::moveRawAbsolute(double v, double tip, double tilt)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    // LFAST::CommsMessage newMsg;
    // newMsg.addKeyValuePair<std::string>("Adjusting Tip/tilt Absolute", "$OK^");
    // commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // Convert x/y inputs to absulute stepper positions relative to zero position (Produces results in mm)
    double Adistance = (281.3 * sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0 * sin(tilt) - 35160.0 * cos(tilt) * sin(tip))) / (cos(tip) * cos(tilt));
    double Cdistance = (-0.004 * (60900.0 * sin(tilt) + 35160.0 * cos(tilt) * sin(tip))) / (cos(tip) * cos(tilt));

    // Convert Distance to steps (0.003mm per step??)
    int Asteps = Adistance / (MM_PER_STEP);
    int Bsteps = Bdistance / (MM_PER_STEP);
    int Csteps = Cdistance / (MM_PER_STEP);

    // Set directions based on desired positions relative to current positions of steppers.
    if (A.currentPosition() > Asteps)
    {
        A.setSpeed(-v);
    }
    else
    {
        A.setSpeed(v);
    }
    if (B.currentPosition() > Bsteps)
    {
        B.setSpeed(-v);
    }
    else
    {
        B.setSpeed(v);
    }
    if (C.currentPosition() > Csteps)
    {
        C.setSpeed(-v);
    }
    else
    {
        C.setSpeed(v);
    }

    while ((A.currentPosition() != Asteps) || (B.currentPosition() != Bsteps) || (C.currentPosition() != Csteps))
    {

        if (A.currentPosition() != Asteps)
        {
            A.runSpeed();
        }
        if (B.currentPosition() != Bsteps)
        {
            B.runSpeed();
        }
        if (C.currentPosition() != Csteps)
        {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

// Move each axis with velocity V X,Y units from the current position In the above commands,
// V, X and Y are vectors of length 3. Velocity is in units of radians per second, X,Y are milliradians.
// Velocity input as rad/ sec, converted to steps / sec
void PrimaryMirrorControl::moveRelative(double v, double tip, double tilt)
{
    // Convert v (rad / second) to steps / second
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel
    moveRawRelative(v, tip, tilt);               // linear velocity / (1 step / 3 um) = steps / sec
}

// Move each axis with velocity V X,Y units from the current position to achieve desired tip/tilt(radians) relative position
// Velocity input as steps / sec
void PrimaryMirrorControl::moveRawRelative(double v, double tip, double tilt)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    // LFAST::CommsMessage newMsg;
    // newMsg.addKeyValuePair<std::string>("Adjusting Tip/tilt Relative", "$OK^");
    // commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // Convert x/y inputs to absulute stepper positions relative to zero position (Produces results in mm)
    double Adistance = (281.3 * sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0 * sin(tilt) - 35160.0 * cos(tilt) * sin(tip))) / (cos(tip) * cos(tilt));
    double Cdistance = (-0.004 * (60900.0 * sin(tilt) + 35160.0 * cos(tilt) * sin(tip))) / (cos(tip) * cos(tilt));

    // Convert Distance to steps (MM_PER_STEP mm per step??)
    int Asteps = Adistance / (MM_PER_STEP);
    int Bsteps = Bdistance / (MM_PER_STEP);
    int Csteps = Cdistance / (MM_PER_STEP);

    char debugMsg[100]{0};
    sprintf(debugMsg, "A Steps: %d\tB Steps: %d\tC Steps: %d", Asteps, Bsteps, Csteps);
    cli->addDebugMessage(debugMsg);

    A.moveTo(A.currentPosition() + Asteps);
    if (Asteps < 0)
    {
        A.setSpeed(-v);
    }
    else
    {
        A.setSpeed(v);
    }
    B.moveTo(B.currentPosition() + Bsteps);
    if (Bsteps < 0)
    {
        B.setSpeed(-v);
    }
    else
    {
        B.setSpeed(v);
    }
    C.moveTo(C.currentPosition() + Csteps);
    if (Csteps < 0)
    {
        C.setSpeed(-v);
    }
    else
    {
        C.setSpeed(v);
    }

    std::memset(debugMsg, 0, sizeof(debugMsg));
    sprintf(debugMsg, "To Go: A: %ld\tB: %ld\tC: %ld", A.distanceToGo(), B.distanceToGo(), C.distanceToGo());
    cli->addDebugMessage(debugMsg);

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0))
    {
        if (A.distanceToGo() != 0)
        {
            A.runSpeed();
        }
        if (B.distanceToGo() != 0)
        {
            B.runSpeed();
        }
        if (C.distanceToGo() != 0)
        {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

// Adjust focus position z(um) at velicity v(rad/sec)
void PrimaryMirrorControl::focusRelative(double v, double z)
{
    // Convert v (rad / second) to steps / second
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel
    focusRelativeRaw(velVal, focusVal);
}

// Adjust focus position z(um) from current position at velicity v(steps/sec)
void PrimaryMirrorControl::focusRelativeRaw(double v, double z)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    // LFAST::CommsMessage newMsg;
    // newMsg.addKeyValuePair<std::string>("Adjusting focus Relative.", "$OK^");
    // commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // Convert Distance to steps 3um per step??
    // z is in microns??
    int steps = z / 3;

    if (z < 0)
    {
        v = -v;
    }
    A.moveTo(A.currentPosition() + steps);
    A.setSpeed(v);
    B.moveTo(B.currentPosition() + steps);
    B.setSpeed(v);
    C.moveTo(C.currentPosition() + steps);
    C.setSpeed(v);

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0))
    {

        if (A.distanceToGo() != 0)
        {
            A.runSpeed();
        }
        if (B.distanceToGo() != 0)
        {
            B.runSpeed();
        }
        if (C.distanceToGo() != 0)
        {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

// v input in (rad/sec), z input as um
void PrimaryMirrorControl::focusAbsolute(double v, double z)
{
    // Convert v (rad / second) to steps / second
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel
    focusRawAbsolute(v, z);
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
    zA = A.currentPosition();
    zB = B.currentPosition();
    zC = C.currentPosition();

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
    // cli->addDebugMessage(z);
    int move = z - zf;
    // convert micron movement back to steps
    move = move / MICRON_PER_STEP; // microns * (1 step / 3 microns)
    // cli->addDebugMessage(move);

    // Equally adust desired actuator movement given desired focus position
    A.moveTo(A.currentPosition() + move);
    A.setSpeed(v);
    B.moveTo(B.currentPosition() + move);
    B.setSpeed(v);
    C.moveTo(C.currentPosition() + move);
    C.setSpeed(v);

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0))
    {

        if (A.distanceToGo() != 0)
        {
            A.runSpeed();
        }
        if (B.distanceToGo() != 0)
        {
            B.runSpeed();
        }
        if (C.distanceToGo() != 0)
        {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}

void PrimaryMirrorControl::save_current_positions()
{
    unsigned int eeAddr = 1;
    int Aposition = A.currentPosition();
    int Bposition = B.currentPosition();
    int Cposition = C.currentPosition();

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

    A.setCurrentPosition(Aposition);
    B.setCurrentPosition(Bposition);
    C.setCurrentPosition(Cposition);
}

//////////////////////////////////////////////////////////////////////////////////////////
/*Code Below is in work*/
//////////////////////////////////////////////////////////////////////////////////////////
// Static or Dynamic?
void PrimaryMirrorControl::jogMirror(double lst)
{
    while (digitalRead(SW))
    {

        static int indx = 0;
        int xValue = 0;
        int yValue = 0;
        int mapX = 0;
        int mapY = 0;

        for (int i = 0; i < 8; i++)
        {
            xValue = xValue + analogRead(VRx);
            yValue = yValue + analogRead(VRy);
        }
        xValue = (xValue / 8) - 127;
        yValue = (yValue / 8) - 149;
        mapX = map(xValue, 0, 1023, -512, 512);
        mapY = -map(yValue, 0, 1023, -512, 512);
        mapX = mapX / (PI * 4);
        mapY = mapY / (PI * 4);

        double Aspeed = (281.3 * sin(mapX)) / cos(mapX);
        double Bspeed = (0.004 * (60900.0 * sin(mapY) - 35160.0 * cos(mapY) * sin(mapX))) / (cos(mapX) * cos(mapY));
        double Cspeed = (-0.004 * (60900.0 * sin(mapY) + 35160.0 * cos(mapY) * sin(mapX))) / (cos(mapX) * cos(mapY));

        if (!(indx % 1000))
        {
            // Serial.print("X: ");
            // cli->addDebugMessage(xValue, DEC);
            // Serial.print("Y: ");
            // cli->addDebugMessage(yValue, DEC);
            // Serial.print("mapX: ");
            // cli->addDebugMessage(mapX, DEC);
            // Serial.print("mapY: ");
            // cli->addDebugMessage(mapY, DEC);
            // Serial.print("A: ");
            // cli->addDebugMessage(Aspeed, DEC);
            // Serial.print("B: ");
            // cli->addDebugMessage(Bspeed, DEC);
            // Serial.print("C: ");
            // cli->addDebugMessage(Cspeed, DEC);
        }

        indx++;

        if ((mapX > 200) || (mapX < -200) || (mapY > 200) || (mapY < -200))
        {
            A.setSpeed(Aspeed);
            A.run();
            B.setSpeed(Bspeed);
            B.run();
            C.setSpeed(Cspeed);
            C.run();
        }
        // delay(10);
    }
}