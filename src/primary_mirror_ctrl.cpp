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
    if (pmc.isEnabled())
        pmc.pingMirrorControlStateMachine();
    interrupts();
}

void PrimaryMirrorControl::limitSwitch_A_ISR()
{
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(A_LIMIT_SW_PIN));
    PrimaryMirrorControl &pmc = PrimaryMirrorControl::getMirrorController();
    pmc.limitSwitchHandler(LFAST::PMC::MOTOR_A);
    interrupts();
}
void PrimaryMirrorControl::limitSwitch_B_ISR()
{
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(B_LIMIT_SW_PIN));
    PrimaryMirrorControl &pmc = PrimaryMirrorControl::getMirrorController();
    if (pmc.isEnabled())
        pmc.limitSwitchHandler(LFAST::PMC::MOTOR_B);
    interrupts();
}
void PrimaryMirrorControl::limitSwitch_C_ISR()
{
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(C_LIMIT_SW_PIN));
    PrimaryMirrorControl &pmc = PrimaryMirrorControl::getMirrorController();
    pmc.limitSwitchHandler(LFAST::PMC::MOTOR_C);
    interrupts();
}
PrimaryMirrorControl::PrimaryMirrorControl()
{
    controlMode = LFAST::PMC::STOP;
    currentMoveState = IDLE;
    currentHomingState = INITIALIZE;
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
    Stepper_A.setMaxSpeed(STEPPER_MAX_SPEED);     // Steps per second
    Stepper_A.setAcceleration(STEPPER_MAX_ACCEL); // Steps per second per second

    Stepper_B.setMaxSpeed(STEPPER_MAX_SPEED);
    Stepper_B.setAcceleration(STEPPER_MAX_ACCEL);

    Stepper_C.setMaxSpeed(STEPPER_MAX_SPEED);
    Stepper_C.setAcceleration(STEPPER_MAX_ACCEL);

    steppers.addStepper(Stepper_A);
    steppers.addStepper(Stepper_B);
    steppers.addStepper(Stepper_C);

    pinMode(STEP_ENABLE_PIN, OUTPUT);

    pinMode(A_LIMIT_SW_PIN, INPUT_PULLUP);
    pinMode(B_LIMIT_SW_PIN, INPUT_PULLUP);
    pinMode(C_LIMIT_SW_PIN, INPUT_PULLUP);

    // Global stepper enable pin, high to diable drivers
    enableLimitSwitchInterrupts();
    // Initialize Timer
    Timer1.initialize(UPDATE_PRD_US);
    Timer1.stop();
    Timer1.attachInterrupt(primaryMirrorControl_ISR);
}

void PrimaryMirrorControl::enableLimitSwitchInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(A_LIMIT_SW_PIN), limitSwitch_A_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(B_LIMIT_SW_PIN), limitSwitch_B_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(C_LIMIT_SW_PIN), limitSwitch_C_ISR, FALLING);
}

void PrimaryMirrorControl::setMoveNotifierFlag(volatile bool *flagPtr)
{
    moveNotifierFlagPtr = flagPtr;
}
void PrimaryMirrorControl::setHomingCompleteNotifierFlag(volatile bool *flagPtr)
{
    homeNotifierFlagPtr = flagPtr;
}
void PrimaryMirrorControl::pingMirrorControlStateMachine()
{
    // tip/tilt/focus adustment control parsing
    bool moveCompleteFlag = false;
    static MOVE_STATE prevMoveState = IDLE;
    static uint32_t counter = 0;

    switch (currentMoveState)
    {
    case IDLE:
        if (checkForNewCommand())
        {
            currentMoveState = NEW_MOVE_CMD;
        }
        break;
    case NEW_MOVE_CMD:
        enableLimitSwitchInterrupts();
        currentMoveState = MOVE_IN_PROGRESS;
        updateStepperCommands();
        // Intentional fall-through
    case MOVE_IN_PROGRESS:
        moveCompleteFlag = pingSteppers();
        saveStepperPositionsToEeprom();
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
        break;
    case MOVE_COMPLETE:
        if (moveNotifierFlagPtr != nullptr)
            *moveNotifierFlagPtr = true;
        currentMoveState = IDLE;
        // Timer1.stop();
    case LIMIT_SW_DETECT:
        stopNow();
        currentMoveState = IDLE;
        break;
    case HOMING_IS_ACTIVE:
        bool homingComplete = pingHomingRoutine();

        if (homingComplete)
            currentMoveState = IDLE;
        break;
    }
    if (counter++ >= TERM_UPDATE_COUNT)
    {
        updateFeedbackFields();
        counter = 0;
    }
    if (prevMoveState != currentMoveState)
    {
        updateStatusFields();
        prevMoveState = currentMoveState;
    }
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
bool PrimaryMirrorControl::isHomingInProgress()
{
    return currentMoveState == HOMING_IS_ACTIVE;
}
void PrimaryMirrorControl::enableControlInterrupt()
{
    Timer1.start();
}

// Functions to update necessary control variables
void PrimaryMirrorControl::copyShadowToActive()
{
    CommandStates_Eng = ShadowCommandStates_Eng;
}

void PrimaryMirrorControl::setControlMode(uint8_t mode)
{
    controlMode = mode;
    // if (controlMode == PMC::RELATIVE)
    // ShadowCommandStates_Eng.reset();
}

void PrimaryMirrorControl::setTipTarget(double tgt)
{
    if (controlMode == PMC::RELATIVE)
        ShadowCommandStates_Eng.TIP_POS_ENG = ShadowCommandStates_Eng.TIP_POS_ENG + tgt;
    else
        ShadowCommandStates_Eng.TIP_POS_ENG = tgt;

    tipUpdated = true;
    // cli->printfDebugMessage("TargetTip = %6.4f", CommandStates_Eng.TIP_POS_ENG);
}

void PrimaryMirrorControl::setTiltTarget(double tgt)
{
    if (controlMode == PMC::RELATIVE)
        ShadowCommandStates_Eng.TILT_POS_ENG = ShadowCommandStates_Eng.TILT_POS_ENG + tgt;
    else
        ShadowCommandStates_Eng.TILT_POS_ENG = tgt;

    tiltUpdated = true;
    // cli->printfDebugMessage("TargetTilt = %6.4f", CommandStates_Eng.TILT_POS_ENG);
}

void PrimaryMirrorControl::setFocusTarget(double tgt)
{
    double focus_tgt_presat;
    if (controlMode == PMC::RELATIVE)
        focus_tgt_presat = ShadowCommandStates_Eng.FOCUS_POS_ENG + tgt;
    else
        focus_tgt_presat = tgt;

    double focus_tgt_post_sat = saturate(focus_tgt_presat, MIN_STROKE_MICRON, MAX_STROKE_MICRON);
    ShadowCommandStates_Eng.FOCUS_POS_ENG = focus_tgt_post_sat;
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
    // Intentionally disregarding acceleration limits etc...
    // This thing moves too slowly to worry about it
    // Stepper_A.stop();
    // Stepper_B.stop();
    // Stepper_C.stop();
    currentMoveState = IDLE;
    currentHomingState = INITIALIZE;
    controlMode = PMC::STOP;

    Stepper_A.moveTo(Stepper_A.currentPosition());
    Stepper_B.moveTo(Stepper_B.currentPosition());
    Stepper_C.moveTo(Stepper_C.currentPosition());

    saveStepperPositionsToEeprom();
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void PrimaryMirrorControl::updateStepperCommands()
{
    // Convert Distance to steps (0.003mm per step??)
    A_cmdSteps = 0;
    B_cmdSteps = 0;
    C_cmdSteps = 0;
    CommandStates_Eng.getMotorPosnCommands(&A_cmdSteps, &B_cmdSteps, &C_cmdSteps);

#if ENABLE_TERMINAL_UPDATES
    cli->printfDebugMessage("Step Commands: [A/B/C]: %d, %d, %d", A_cmdSteps, B_cmdSteps, C_cmdSteps);
#endif
    long stepperCmdVector[3]{A_cmdSteps, B_cmdSteps, C_cmdSteps};
    steppers.moveTo(stepperCmdVector);

    updateCommandFields();
}

bool PrimaryMirrorControl::pingSteppers()
{
    bool moveCompleteFlag = false;

    // multistepper run function returns true if any of the motors are still moving
    if (!steppers.run())
    {
        moveCompleteFlag = true;
    }
    return moveCompleteFlag;
}
bool PrimaryMirrorControl::pingHomingRoutine()
{
    static uint32_t waitStartCount = 0;
    uint32_t waitCounter = 0;

    bool homingComplete = false;
    switch (currentHomingState)
    {
    case INITIALIZE:
        limitFound_A = false;
        limitFound_B = false;
        limitFound_C = false;
        Stepper_A.setSpeed(-homingSpeedStepsPerSec);
        Stepper_B.setSpeed(-homingSpeedStepsPerSec);
        Stepper_C.setSpeed(-homingSpeedStepsPerSec);
        currentHomingState = HOMING_STEP_1;
        updateStatusFields();
        break;
    case HOMING_STEP_1:
        // Quick move until all endstops are hit
        if (!limitFound_A)
            Stepper_A.runSpeed();
        if (!limitFound_B)
            Stepper_B.runSpeed();
        if (!limitFound_C)
            Stepper_C.runSpeed();
        // delay(1);
        if (limitFound_A && limitFound_B && limitFound_C)
        {
            saveStepperPositionsToEeprom();
            currentHomingState = HOMING_STEP_2;
            waitStartCount = millis();
            updateStatusFields();
        }
        break;
    case HOMING_STEP_2:
        // Short pause
        waitCounter = millis();
        if ((waitCounter - waitStartCount) > 1000)
        {
            Stepper_A.setSpeed(homingSpeedStepsPerSec * 0.5);
            Stepper_B.setSpeed(homingSpeedStepsPerSec * 0.5);
            Stepper_C.setSpeed(homingSpeedStepsPerSec * 0.5);
            currentHomingState = HOMING_STEP_3;
        }
        break;
    case HOMING_STEP_3:
        // Slow Move forward until endstops are cleared
        if (Stepper_A.currentPosition() < 500 * MICROSTEP_DIVIDER)
        {
            Stepper_A.runSpeed();
            Stepper_B.runSpeed();
            Stepper_C.runSpeed();
        }
        else
        {
            if ((digitalRead(A_LIMIT_SW_PIN)) == HIGH &&
                (digitalRead(B_LIMIT_SW_PIN)) == HIGH &&
                (digitalRead(C_LIMIT_SW_PIN)) == HIGH)
            {
                limitFound_A = false;
                limitFound_B = false;
                limitFound_C = false;
                enableLimitSwitchInterrupts();
                currentHomingState = HOMING_STEP_4;
                waitStartCount = millis();
                updateStatusFields();
            }
        }
        break;
    case HOMING_STEP_4:
        // Shorter pause
        waitCounter = millis();
        if ((waitCounter - waitStartCount) > 300)
        {
            Stepper_A.setSpeed(homingSpeedStepsPerSec * -0.1);
            Stepper_B.setSpeed(homingSpeedStepsPerSec * -0.1);
            Stepper_C.setSpeed(homingSpeedStepsPerSec * -0.1);
            currentHomingState = HOMING_STEP_5;
        }
        break;
    case HOMING_STEP_5:
        // Very slow move backwards until endstops are hit again
        if (!limitFound_A)
            Stepper_A.runSpeed();
        if (!limitFound_B)
            Stepper_B.runSpeed();
        if (!limitFound_C)
            Stepper_C.runSpeed();
        // delay(1);
        if (limitFound_A && limitFound_B && limitFound_C)
        {
            enableLimitSwitchInterrupts();
            saveStepperPositionsToEeprom();
            if (homeNotifierFlagPtr != nullptr)
                *homeNotifierFlagPtr = true;
            ShadowCommandStates_Eng.reset();
            CommandStates_Eng.reset();
            homingComplete = true;
        }
        break;
    }

    return homingComplete;
}
void PrimaryMirrorControl::enableSteppers(bool doEnable)
{
    if (doEnable)
    {
        digitalWrite(STEP_ENABLE_PIN, ENABLE_STEPPER);
        cli->updatePersistentField(DeviceName, STEPPERS_ENABLED, "True");
    }
    else
    {
        currentMoveState = IDLE;
        controlMode = PMC::STOP;
        digitalWrite(STEP_ENABLE_PIN, DISABLE_STEPPER);
        cli->updatePersistentField(DeviceName, STEPPERS_ENABLED, "False");
    }
    if (cli != nullptr)

        steppersEnabled = doEnable;
}
// Move all actuators to home positions at velocity V (steps/sec)
void PrimaryMirrorControl::goHome(volatile double homingSpeed)
{
    homingSpeedStepsPerSec = (homingSpeed * MIRROR_RADIUS) / (MICRON_PER_STEP);
    currentMoveState = HOMING_IS_ACTIVE;
    currentHomingState = INITIALIZE;
    controlMode = PMC::RELATIVE;
}
void PrimaryMirrorControl::limitSwitchHandler(uint16_t motor)
{
    // For de-bounce
    delayMicroseconds(500);
    if (motor == LFAST::PMC::MOTOR_A)
    {
        if (currentMoveState != HOMING_IS_ACTIVE)
            attachInterrupt(digitalPinToInterrupt(A_LIMIT_SW_PIN), limitSwitch_A_ISR, FALLING);
        cli->printDebugMessage("A Limit Switch Detected");
        Stepper_A.setCurrentPosition(-0.5*STROKE_STEPS);
        limitFound_A = true;
    }
    else if (motor == LFAST::PMC::MOTOR_B)
    {
        if (currentMoveState != HOMING_IS_ACTIVE)
            attachInterrupt(digitalPinToInterrupt(B_LIMIT_SW_PIN), limitSwitch_B_ISR, FALLING);
        cli->printDebugMessage("B Limit Switch Detected");
        Stepper_B.setCurrentPosition(-0.5*STROKE_STEPS);
        limitFound_B = true;
    }
    else if (motor == LFAST::PMC::MOTOR_C)
    {
        if (currentMoveState != HOMING_IS_ACTIVE)
            attachInterrupt(digitalPinToInterrupt(C_LIMIT_SW_PIN), limitSwitch_C_ISR, FALLING);
        cli->printDebugMessage("C Limit Switch Detected");
        Stepper_C.setCurrentPosition(-0.5*STROKE_STEPS);
        limitFound_C = true;
    }

    if (currentMoveState != HOMING_IS_ACTIVE)
    {
        currentMoveState = LIMIT_SW_DETECT;
    }
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

double PrimaryMirrorControl::getStepperPosition(uint8_t motor)
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

void PrimaryMirrorControl::saveStepperPositionsToEeprom()
{
    int Aposition = Stepper_A.currentPosition();
    int Bposition = Stepper_B.currentPosition();
    int Cposition = Stepper_C.currentPosition();
    EEPROM.put(EEPROM_ADDR_STEPPER_A_POS, Aposition);
    EEPROM.put(EEPROM_ADDR_STEPPER_B_POS, Bposition);
    EEPROM.put(EEPROM_ADDR_STEPPER_C_POS, Cposition);
}

void PrimaryMirrorControl::resetPositionsInEeprom()
{
    EEPROM.put(EEPROM_ADDR_STEPPER_A_POS, 0);
    EEPROM.put(EEPROM_ADDR_STEPPER_B_POS, 0);
    EEPROM.put(EEPROM_ADDR_STEPPER_C_POS, 0);
    cli->printDebugMessage("Resetting eeprom positions", LFAST::WARNING);
}

void PrimaryMirrorControl::loadCurrentPositionsFromEeprom()
{
    int Aposition = 0;
    int Bposition = 0;
    int Cposition = 0;

    EEPROM.get(EEPROM_ADDR_STEPPER_A_POS, Aposition);
    EEPROM.get(EEPROM_ADDR_STEPPER_B_POS, Bposition);
    EEPROM.get(EEPROM_ADDR_STEPPER_C_POS, Cposition);

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
    cli->addPersistentField(this->DeviceName, "[ENABLED?]", STEPPERS_ENABLED);
    cli->addPersistentField(this->DeviceName, "[STEPPER A]", STEPPER_A_FB);
    cli->addPersistentField(this->DeviceName, "[STEPPER B]", STEPPER_B_FB);
    cli->addPersistentField(this->DeviceName, "[STEPPER C]", STEPPER_C_FB);
    cli->addPersistentField(this->DeviceName, "[TIP EST]", TIP_FB_ROW);
    cli->addPersistentField(this->DeviceName, "[TILT EST]", TILT_FB_ROW);
    cli->addPersistentField(this->DeviceName, "[FOCUS EST]", FOCUS_FB_ROW);
    updateStatusFields();
}

void PrimaryMirrorControl::updateStatusFields()
{
#if ENABLE_TERMINAL_UPDATES
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
    case LIMIT_SW_DETECT:
        cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "LIMIT_SW_DETECT");
        break;
    case HOMING_IS_ACTIVE:
        switch (currentHomingState)
        {
        case INITIALIZE:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (INIT)");
            break;
        case HOMING_STEP_1:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (STEP 1)");
            break;
        case HOMING_STEP_2:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (STEP 2)");
            break;
        case HOMING_STEP_3:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (STEP 3)");
            break;
        case HOMING_STEP_4:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (STEP 4)");
            break;
        case HOMING_STEP_5:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING (STEP 5)");
            break;
        }

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
#endif
}

void PrimaryMirrorControl::updateCommandFields()
{
#if ENABLE_TERMINAL_UPDATES
    cli->updatePersistentField(DeviceName, TIP_ROW, CommandStates_Eng.TIP_POS_ENG);
    cli->updatePersistentField(DeviceName, TILT_ROW, CommandStates_Eng.TILT_POS_ENG);
    cli->updatePersistentField(DeviceName, FOCUS_ROW, CommandStates_Eng.FOCUS_POS_ENG);
#endif
}

void PrimaryMirrorControl::updateFeedbackFields()
{
#if ENABLE_TERMINAL_UPDATES
    auto aPos = Stepper_A.currentPosition();
    auto bPos = Stepper_B.currentPosition();
    auto cPos = Stepper_C.currentPosition();
    MotorStates motorStates(aPos, bPos, cPos);
    double tipEst, tiltEst, focusEst;
    motorStates.getTipTiltFocusFeedback(&tipEst, &tiltEst, &focusEst);
    cli->updatePersistentField(DeviceName, STEPPER_A_FB, aPos);
    cli->updatePersistentField(DeviceName, STEPPER_B_FB, bPos);
    cli->updatePersistentField(DeviceName, STEPPER_C_FB, cPos);
    cli->updatePersistentField(DeviceName, TIP_FB_ROW, tipEst);
    cli->updatePersistentField(DeviceName, TILT_FB_ROW, tiltEst);
    cli->updatePersistentField(DeviceName, FOCUS_FB_ROW, focusEst);
#endif
}