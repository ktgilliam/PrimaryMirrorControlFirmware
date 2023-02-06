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
#if EEPROM_ENABLED
#include <EEPROM.h>
#endif
#if FRAM_ENABLED
#include <FRAM.h>
#endif
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

#if FRAM_ENABLED
FRAM fram;
uint32_t start;
uint32_t stop;
uint32_t sizeInBytes = 0;
#endif

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
    {
        // TOGGLE_DEBUG_PIN();
        pmc.pingMirrorControlStateMachine();
        // delay(1);
        // TOGGLE_DEBUG_PIN();
    }
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

void PrimaryMirrorControl::initializeNVRAM()
{
#if FRAM_ENABLED
    int rv = fram.begin(0x50);
    if (rv != 0)
    {
        cli->printfDebugMessage("FRAM Init failed: %d", rv);
    }
    else
    {
        // cli->printfDebugMessage(__FUNCTION__);
        cli->printfDebugMessage("FRAM ID: %d, Prod: %d, MemKb: ", fram.getManufacturerID(), fram.getProductID(), fram.getSize());
    }
#endif
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
    this->enableSteppers(false);

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
        SET_DEBUG_PIN();
        moveCompleteFlag = pingSteppers();
        // saveStepperPositionsToNVRAM();
        CLEAR_DEBUG_PIN();
        if (moveCompleteFlag)
        {
            cli->printDebugMessage("Move complete.");
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
        saveStepperPositionsToNVRAM();
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
    // if (focusUpdated)
    //     cli->printfDebugMessage("Focus shadow: %.2f", ShadowCommandStates_Eng.FOCUS_POS_MM);
    CommandStates_Eng = ShadowCommandStates_Eng;
    // if (focusUpdated)
    //     cli->printfDebugMessage("Focus active: %.2f", CommandStates_Eng.FOCUS_POS_MM);
}

void PrimaryMirrorControl::setControlMode(uint8_t mode)
{
    controlMode = mode;
}

void PrimaryMirrorControl::setTipTarget(double tgt_as)
{
    double tgt_rad_presat;
    if (controlMode == PMC::RELATIVE)
    {
        if (currentMoveState != MOVE_IN_PROGRESS)
        {
            // See comments in setFocusTarget
            tgt_rad_presat = ShadowCommandStates_Eng.tip() + (tgt_as * RAD_PER_AS);
            tipUpdated = true;
        }
        else
        {
            tipUpdated = false;
        }
    }
    else
    {
        tgt_rad_presat = (tgt_as * RAD_PER_AS);
        tipUpdated = true;
    }
    // cli->printfDebugMessage("TIP UPDATE: %.8f", tgt_rad_presat * URAD_PER_RAD);
    if (tipUpdated)
    {
        // TODO: Add angle saturation to limit command to mechanical range.
        ShadowCommandStates_Eng.setTip(tgt_rad_presat);
    }
}

void PrimaryMirrorControl::setTiltTarget(double tgt_as)
{
    double tgt_rad_presat;
    if (controlMode == PMC::RELATIVE)
    {
        if (currentMoveState != MOVE_IN_PROGRESS)
        {
            tiltUpdated = true;
            // See comments in setFocusTarget
            tgt_rad_presat = ShadowCommandStates_Eng.tilt() + (tgt_as * RAD_PER_AS);
        }
        else
        {
            tiltUpdated = false;
        }
    }
    else
    {
        tiltUpdated = true;
        tgt_rad_presat = (tgt_as * RAD_PER_AS);
    }
    // cli->printfDebugMessage("TILT UPDATE: %.8f", tgt_rad_presat* URAD_PER_RAD);
    if (tiltUpdated)
    {
        // TODO: Add angle saturation to limit command to mechanical range.
        ShadowCommandStates_Eng.setTilt(tgt_rad_presat);
    }
}

void PrimaryMirrorControl::setFocusTarget(double tgt_um)
{
    double focus_tgt_presat;
    if (controlMode == PMC::RELATIVE)
    {
        if (currentMoveState != MOVE_IN_PROGRESS)
        {
            // In order to process relative commands while one is in progress,
            // I need to finish getting the motor->tip/tilt/focus transforms correct
            // So we can see what the current position is before adding to it.
            // For now, relative commands will only get processed if there is no move in progress.
            focus_tgt_presat = ShadowCommandStates_Eng.focus() + tgt_um;
            focusUpdated = true;
        }
        else
        {
            focusUpdated = false;
        }
    }
    else
    {
        focus_tgt_presat = tgt_um;
        focusUpdated = true;
    }
    if (focusUpdated)
    {
        ShadowCommandStates_Eng.setFocus(focus_tgt_presat);
        // ShadowCommandStates_Eng.FOCUS_POS_MM = focus_tgt_presat;
        // cli->printfDebugMessage("TargetFocus = %6.4f", CommandStates_Eng.focus());
    }
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

    saveStepperPositionsToNVRAM();
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

    // #if ENABLE_TERMINAL_UPDATES
    //     cli->printfDebugMessage("Command update [t/t/f]: %.2f, %.2f, %.2f", CommandStates_Eng.TIP_POS_RAD, CommandStates_Eng.TILT_POS_RAD, CommandStates_Eng.FOCUS_POS_MM);
    // #endif
    CommandStates_Eng.getMotorPosnCommands(&A_cmdSteps, &B_cmdSteps, &C_cmdSteps);
    long stepperCmdVector[3]{A_cmdSteps, B_cmdSteps, C_cmdSteps};
    steppers.moveTo(stepperCmdVector);
    updateCommandFields();
}

bool PrimaryMirrorControl::pingSteppers()
{
    bool moveCompleteFlag = false;
    if (!steppers.run())
    {
#if ENABLE_TERMINAL_UPDATES

#endif
        moveCompleteFlag = true;
    }
    else
    {
        // cli->printDebugMessage("Move in progress.");
    }
    return moveCompleteFlag;
}
bool PrimaryMirrorControl::pingHomingRoutine()
{
    bool homingComplete = false;
    static HOMING_STATE prevHomingState = INITIALIZE;
    long homingStep1Cmds[3]{-2*FULL_STROKE_STEPS, -2*FULL_STROKE_STEPS, -2*FULL_STROKE_STEPS};
    long homingStep2Cmds[3]{0, 0, 0};

    switch (currentHomingState)
    {
    case INITIALIZE:
        limitFound_A = false;
        limitFound_B = false;
        limitFound_C = false;
        steppers.moveTo(homingStep1Cmds);
        currentHomingState = HOMING_STEP_1;
        updateStatusFields();
        break;
    case HOMING_STEP_1:
        // Move until bottomFound message is sent from GUI
        if (!steppers.run())
        {
            // TODO: Something is wrong
        }
        // updateStatusFields();
        break;
    case HOMING_STEP_2:
        // Bottom found
        Stepper_A.setCurrentPosition(STROKE_BOTTOM_STEPS);
        Stepper_B.setCurrentPosition(STROKE_BOTTOM_STEPS);
        Stepper_C.setCurrentPosition(STROKE_BOTTOM_STEPS);
        this->saveStepperPositionsToNVRAM();
        steppers.moveTo(homingStep2Cmds);
        currentHomingState = HOMING_STEP_3;
        break;

    case HOMING_STEP_3:
        // Return to middle of travel

        if (!steppers.run())
        {
#if ENABLE_TERMINAL_UPDATES

#endif
            if (homeNotifierFlagPtr != nullptr)
                *homeNotifierFlagPtr = true;
            ShadowCommandStates_Eng.resetToHomed();
            CommandStates_Eng.resetToHomed();
            // this->resetPositionsInNVRAM();
            homingComplete = true;
        }
        else
        {
            // cli->printDebugMessage("still trying...");
            homingComplete = false;
        }
        break;
    }
    if (prevHomingState != currentHomingState)
    {
        updateStatusFields();
        prevHomingState = currentHomingState;
    }

    return homingComplete;
}
void PrimaryMirrorControl::enableSteppers(bool doEnable)
{
    if (doEnable)
    {
        digitalWrite(STEP_ENABLE_PIN, ENABLE_STEPPER);
        if (cli != nullptr)
            cli->updatePersistentField(DeviceName, STEPPERS_ENABLED, "True");
    }
    else
    {
        currentMoveState = IDLE;
        controlMode = PMC::STOP;
        digitalWrite(STEP_ENABLE_PIN, DISABLE_STEPPER);
        if (cli != nullptr)
            cli->updatePersistentField(DeviceName, STEPPERS_ENABLED, "False");
    }
    if (cli != nullptr)
        steppersEnabled = doEnable;
}
// Move all actuators to home positions at velocity V (steps/sec)
void PrimaryMirrorControl::goHome(volatile double homingSpeed)
{
    homingSpeedStepsPerSec = (homingSpeed * MIRROR_RADIUS) / (UM_PER_STEP);
    currentMoveState = HOMING_IS_ACTIVE;
    currentHomingState = INITIALIZE;
    controlMode = PMC::RELATIVE;
}

void PrimaryMirrorControl::bottomFound()
{
    if (currentMoveState == HOMING_IS_ACTIVE && currentHomingState == HOMING_STEP_1)
    {
        currentHomingState = HOMING_STEP_2;
    }
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
        Stepper_A.setCurrentPosition(STROKE_BOTTOM_STEPS);
        limitFound_A = true;
    }
    else if (motor == LFAST::PMC::MOTOR_B)
    {
        if (currentMoveState != HOMING_IS_ACTIVE)
            attachInterrupt(digitalPinToInterrupt(B_LIMIT_SW_PIN), limitSwitch_B_ISR, FALLING);
        cli->printDebugMessage("B Limit Switch Detected");
        Stepper_B.setCurrentPosition(STROKE_BOTTOM_STEPS);
        limitFound_B = true;
    }
    else if (motor == LFAST::PMC::MOTOR_C)
    {
        if (currentMoveState != HOMING_IS_ACTIVE)
            attachInterrupt(digitalPinToInterrupt(C_LIMIT_SW_PIN), limitSwitch_C_ISR, FALLING);
        cli->printDebugMessage("C Limit Switch Detected");
        Stepper_C.setCurrentPosition(STROKE_BOTTOM_STEPS);
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

void PrimaryMirrorControl::saveStepperPositionsToNVRAM()
{
    int Aposition = Stepper_A.currentPosition();
    int Bposition = Stepper_B.currentPosition();
    int Cposition = Stepper_C.currentPosition();
#if EEPROM_ENABLED
    EEPROM.put(NVRAM_ADDR_STEPPER_A_POS, Aposition);
    EEPROM.put(NVRAM_ADDR_STEPPER_B_POS, Bposition);
    EEPROM.put(NVRAM_ADDR_STEPPER_C_POS, Cposition);
#endif
#if FRAM_ENABLED
    fram.write32(NVRAM_ADDR_STEPPER_A_POS, Aposition);
    fram.write32(NVRAM_ADDR_STEPPER_B_POS, Bposition);
    fram.write32(NVRAM_ADDR_STEPPER_C_POS, Cposition);
#endif
}

void PrimaryMirrorControl::resetPositionsInNVRAM(uint32_t A_pos, uint32_t B_pos, uint32_t C_pos)
{
#if EEPROM_ENABLED
    EEPROM.put(NVRAM_ADDR_STEPPER_A_POS, A_pos);
    EEPROM.put(NVRAM_ADDR_STEPPER_B_POS, B_pos);
    EEPROM.put(NVRAM_ADDR_STEPPER_C_POS, C_pos);
#endif
#if FRAM_ENABLED
    fram.write32(NVRAM_ADDR_STEPPER_A_POS, A_pos);
    fram.write32(NVRAM_ADDR_STEPPER_B_POS, B_pos);
    fram.write32(NVRAM_ADDR_STEPPER_C_POS, C_pos);
#endif
    cli->printDebugMessage("Resetting positions in NVRAM", LFAST::WARNING_MESSAGE);
}

void PrimaryMirrorControl::loadCurrentPositionsFromNVRAM()
{
    int Aposition = 0;
    int Bposition = 0;
    int Cposition = 0;
#if EEPROM_ENABLED
    EEPROM.get(NVRAM_ADDR_STEPPER_A_POS, Aposition);
    EEPROM.get(NVRAM_ADDR_STEPPER_B_POS, Bposition);
    EEPROM.get(NVRAM_ADDR_STEPPER_C_POS, Cposition);
#endif
#if FRAM_ENABLED
    Aposition = fram.read32(NVRAM_ADDR_STEPPER_A_POS);
    Bposition = fram.read32(NVRAM_ADDR_STEPPER_B_POS);
    Cposition = fram.read32(NVRAM_ADDR_STEPPER_C_POS);
#endif
    cli->printfDebugMessage("NVRAM Load [A/B/C]: %d, %d, %d", Aposition, Bposition, Cposition);
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
    cli->addPersistentField(this->DeviceName, "[TIP CMD]", TIP_CMD_ROW);
    cli->addPersistentField(this->DeviceName, "[TILT CMD]", TILT_CMD_ROW);
    cli->addPersistentField(this->DeviceName, "[FOCUS CMD]", FOCUS_CMD_ROW);
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
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING 1 (QUICK REVERSE)");
            break;
        case HOMING_STEP_2:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING 2 (BOTTOM FOUND)");
            break;
        case HOMING_STEP_3:
            cli->updatePersistentField(DeviceName, MOVE_SM_STATE_ROW, "HOMING 3 (RETURN TO MIDDLE)");
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
    cli->updatePersistentField(DeviceName, TIP_CMD_ROW, CommandStates_Eng.tip() * RAD_PER_AS*.001, "%.10f urad");
    cli->updatePersistentField(DeviceName, TILT_CMD_ROW, CommandStates_Eng.tilt() * RAD_PER_AS*.001, "%.10f urad");
    cli->updatePersistentField(DeviceName, FOCUS_CMD_ROW, CommandStates_Eng.focus() * UM_PER_MM, "%.10f um");
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
    cli->updatePersistentField(DeviceName, TIP_FB_ROW, tipEst * RAD_PER_AS*.001, "%.10f urad");
    cli->updatePersistentField(DeviceName, TILT_FB_ROW, tiltEst * RAD_PER_AS*.001, "%.10f urad");
    // cli->updatePersistentField(DeviceName, FOCUS_FB_ROW, focusEst);
#endif
}