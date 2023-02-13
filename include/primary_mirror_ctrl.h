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
@brief LFAST prototype Primary Mirror Control Interface function definitions
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror_ctrl.h

Definitions of Primary Mirror Control functions
*/

/*
MoveAbsolute(V, X,Y) – Move each axis with velocity V to an absolute X,Y position with respect to “home”
MoveRelative(V, X,Y) – Move each axis with velocity V X,Y units from the current position In the above commands,
                       V, X and Y are vectors of length 3. Velocity is in units of radians per second, X,Y are milliradians.
MoveRawAbsolute(V, X,Y) – Move each axis with velocity V to an absolute X,Y position with respect to “home”
MoveRawRelative(V, X,Y) – Move each axis with velocity V X,Y units from the current position In the above commands,
                          V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
Home(V) – Move all actuators to home positions at velocity V
FanSpeed(S) – Set the fan speed to a percentage S of full scale
GetStatus() – Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving
GetPositions() – Returns 3 step counts
Stop() – Immediately stops all motion
*/

#ifndef PRIMARY_MIRROR_CONTROL_H
#define PRIMARY_MIRROR_CONTROL_H

#include <Arduino.h>
#include <iostream>
#include <LFAST_Device.h>
#include <TerminalInterface.h>
#include <cmath>
#include <algorithm>

#include <MultiStepper.h>
#include <math_util.h>
#include "teensy41_device.h"
// Setup functions

#define ENABLE_STEPPER LOW
#define DISABLE_STEPPER HIGH

constexpr double MICROSTEP_DIVIDER = 16;
constexpr double MICROSTEP_RATIO = 1.0 / MICROSTEP_DIVIDER;

constexpr double MIRROR_RADIUS_UM = 281288.0;          // Radius of mirror actuator positions in um
constexpr double UM_PER_STEP = 0.2;//3.175 * MICROSTEP_RATIO; // (rounded!) conversion factor of stepper motor steps to vertical movement in um
constexpr double STEPS_PER_UM = 1.0 / UM_PER_STEP;

constexpr double FULL_STROKE_UM = 12700.0;
constexpr double MAX_STROKE_UM = (0.5 * FULL_STROKE_UM);
constexpr double MIN_STROKE_UM = (-0.5 * FULL_STROKE_UM);
constexpr long int FULL_STROKE_STEPS = (long int)(FULL_STROKE_UM / UM_PER_STEP); // 4000*MICROSTEP_DIVIDER
constexpr long int STROKE_TOP_STEPS = (FULL_STROKE_STEPS/2);
constexpr long int STROKE_BOTTOM_STEPS = (-1  * STROKE_TOP_STEPS);
constexpr double STROKE_BOTTOM_UM = ((double)STROKE_BOTTOM_STEPS * UM_PER_STEP);

constexpr double AS_PER_RAD = 648000/M_PI;
constexpr double RAD_PER_AS = 1.0/AS_PER_RAD;


// Mirror coeffs assume microns!!
const double MIRROR_MATH_COEFF_0 = 281300;
const double MIRROR_MATH_COEFF_1 = 140600;
const double MIRROR_MATH_COEFF_2 = 243600;
// Units don't matter for motor math coeffs.
const double MOTOR_MATH_COEFF_0 = 0.001185025075130589607;
const double MOTOR_MATH_COEFF_1 = 0.00205252363836930761;
const double MOTOR_MATH_COEFF_2 = 1.0;


constexpr double UM_PER_MM = 1000.0;
constexpr double MM_PER_UM = 1.0/UM_PER_MM;

// NOTE!! These should be +/- 1.0mm, but useful to make bigger for debugging.
constexpr double FOCUS_MAX_UM = 6.35 * UM_PER_MM;

constexpr double TIP_TILT_STROKE_UM = (FULL_STROKE_UM * 0.5) - FOCUS_MAX_UM;
constexpr double TIP_TILT_MAX_RAD = 0.019017359518293;//atan2(TIP_TILT_STROKE_UM, MIRROR_RADIUS_UM);
// constexpr double TIP_TILT_MAX_DEG = 180/M_PI*TIP_TILT_MAX_RAD;

// PM Control functions
enum PRIMARY_MIRROR_ROWS
{
    BLANK_ROW_0,
    CMD_MODE_ROW,
    TIP_CMD_ROW,
    TILT_CMD_ROW,
    FOCUS_CMD_ROW,
    BLANK_ROW_1,
    TIP_FB_ROW,
    TILT_FB_ROW,
    FOCUS_FB_ROW,
    BLANK_ROW_2,
    STEPPERS_ENABLED,
    MOVE_SM_STATE_ROW,
    STEPPER_A_FB,
    STEPPER_B_FB,
    STEPPER_C_FB,
};

namespace LFAST
{
    namespace PMC
    {
        enum ControlMode
        {
            STOP = 0,
            RELATIVE = 1,
            ABSOLUTE = 2,
        };

        enum UNIT_TYPES
        {
            ENGINEERING = 0,
            STEPS_PER_SEC = 1
        };

        enum AXIS
        {
            TIP = 0,
            TILT = 1,
            FOCUS = 2
        };

        enum DIRECTION
        {
            REVERSE = -1,
            FORWARD = 1
        };

        enum MOTOR_ID
        {
            MOTOR_A = 0,
            MOTOR_B = 1,
            MOTOR_C = 2
        };

    }
};

class MotorStates
{
private:
    vectorX<double, 3> getMirrorVector(const int32_t *am, const int32_t *bm, const int32_t *cm) const
    {
        double A = (double)*am;
        double B = (double)*bm;
        double C = (double)*cm;
        vectorX<double, 3> normalVector;

        constexpr double c[3]{MOTOR_MATH_COEFF_0, MOTOR_MATH_COEFF_1, MOTOR_MATH_COEFF_2};
        normalVector[0] = (c[0] * (B + C)) - 2 * c[0] * A;
        normalVector[1] = c[1] * (C - B);
        normalVector[2] = c[2];
        normalVector.normalize();
        return normalVector;
    }

public:
    MotorStates(int32_t A, int32_t B, int32_t C) : A_steps(A), B_steps(B), C_steps(C) {}

    MotorStates &operator=(MotorStates const &other)
    {
        noInterrupts();
        A_steps = other.A_steps;
        B_steps = other.B_steps;
        C_steps = other.C_steps;
        interrupts();
        return *this;
    }

    int32_t A_steps;
    int32_t B_steps;
    int32_t C_steps;

    void getTipTiltFocusFeedback(double *tip, double *tilt, double *focus)
    {
        auto mirrorVector = getMirrorVector(&A_steps, &B_steps, &C_steps);
        auto norm_XZ = mirrorVector;
        norm_XZ[1] = 0.0;
        norm_XZ.normalize();
        double tipAngle_rad = std::acos(norm_XZ[2]);
        double tiltAngle_rad = std::acos(mirrorVector[2] * std::cos(tipAngle_rad) - mirrorVector[0] * std::sin(tipAngle_rad));
        *tip = tipAngle_rad;
        *tilt = tiltAngle_rad;
        *focus = 0; // TBD, haven't decided best way to do this part yet
    }
};
class MirrorStates
{
private:
    volatile double _TIP_POS_RAD;
    volatile double _TILT_POS_RAD;
    volatile double _FOCUS_POS_UM;
public:
    MirrorStates &operator=(MirrorStates const &other)
    {
        noInterrupts();
        _TIP_POS_RAD = other._TIP_POS_RAD;
        _TILT_POS_RAD = other._TILT_POS_RAD;
        _FOCUS_POS_UM = other._FOCUS_POS_UM;
        interrupts();
        return *this;
    }


    bool setTip(double _tipTgt_rad)
    {
        double tipTgt_sat = constrain(_tipTgt_rad, -TIP_TILT_MAX_RAD, TIP_TILT_MAX_RAD);
        _TIP_POS_RAD = tipTgt_sat;
        return (_tipTgt_rad != tipTgt_sat);
    }
    bool setTilt(double _tiltTgt_rad)
    {
        double tiltTgt_sat = constrain(_tiltTgt_rad, -TIP_TILT_MAX_RAD, TIP_TILT_MAX_RAD);
        _TILT_POS_RAD = tiltTgt_sat;
        return (_tiltTgt_rad != tiltTgt_sat);
    }
    bool setFocus(double _focusTgt_um)
    {
        double focusTgt_sat = constrain(_focusTgt_um, -FOCUS_MAX_UM, FOCUS_MAX_UM);
        _FOCUS_POS_UM = focusTgt_sat;
        return (_focusTgt_um != focusTgt_sat);
    }

    double tip() {return _TIP_POS_RAD;}
    double tilt() {return _TILT_POS_RAD;}
    double focus() {return _FOCUS_POS_UM;}

    bool getMotorPosnCommands(int32_t *a_steps, int32_t *b_steps, int32_t *c_steps) const
    {

        double tanAlpha = std::tan(_TIP_POS_RAD);
        double cosAlpha = std::cos(_TIP_POS_RAD);
        double tanBeta = std::tan(_TILT_POS_RAD);
        double gamma = _FOCUS_POS_UM;

        constexpr double c[3]{MIRROR_MATH_COEFF_0, MIRROR_MATH_COEFF_1, MIRROR_MATH_COEFF_2};
        double a_distance = gamma - (c[0] * tanBeta) / cosAlpha;
        double b_distance = gamma + (c[2] * tanAlpha + c[1] * tanBeta / cosAlpha);
        double c_distance = gamma - (c[2] * tanAlpha + c[1] * tanBeta / cosAlpha);

        int32_t a_steps_presat = (int32_t)(a_distance * STEPS_PER_UM) ;
        int32_t b_steps_presat = (int32_t)(b_distance * STEPS_PER_UM);
        int32_t c_steps_presat = (int32_t)(c_distance * STEPS_PER_UM);
        
        *a_steps = a_steps_presat;
        *b_steps = b_steps_presat;
        *c_steps = c_steps_presat;

        return 0;
    }
    void resetToHomed()
    {
        _TIP_POS_RAD = 0.0;
        _TILT_POS_RAD = 0.0;
        _FOCUS_POS_UM = 0.0;
    }
};

// void updateControlLoop_ISR();

class PrimaryMirrorControl : public LFAST_Device
{
public:
    static PrimaryMirrorControl &getMirrorController();

    virtual ~PrimaryMirrorControl() {}
    void setupPersistentFields() override;
    void updateStatusFields();
    void updateCommandFields();
    void updateFeedbackFields();
    void pingMirrorControlStateMachine();
    void copyShadowToActive();
    void setControlMode(uint8_t moveType);
    void setFanSpeed(unsigned int PWR);
    void setTipTarget(double tgt);
    void setTiltTarget(double tgt);
    void setFocusTarget(double tgt);
    void goHome(volatile double homeSpeed);
    void bottomFound();
    void stopNow();
    bool getStatus(uint8_t motor);
    double getStepperPosition(uint8_t motor);

    void saveStepperPositionsToNVRAM();
    void resetPositionsInNVRAM(uint32_t A_pos = 0, uint32_t B_pos = 0, uint32_t C_pos = 0);
    void loadCurrentPositionsFromNVRAM();
    void enableControlInterrupt();
    void setMoveNotifierFlag(volatile bool *flagPtr);
    void setHomingCompleteNotifierFlag(volatile bool *flagPtr);
    bool checkForNewCommand();
    bool isHomingInProgress();

    void limitSwitchHandler(uint16_t axis);
    void enableSteppers(bool doEnable);
    bool isEnabled() { return steppersEnabled; }
    bool moveInProgress() { return currentMoveState == MOVE_IN_PROGRESS; }
    void initializeNVRAM();

private:
    PrimaryMirrorControl();
    void hardware_setup();
    void enableLimitSwitchInterrupts();
    void updateStepperCommands();
    bool pingSteppers();
    bool pingHomingRoutine();
    MultiStepper *stepperControl;
    MirrorStates CommandStates_Eng;
    MirrorStates ShadowCommandStates_Eng;
    uint8_t controlMode;

    bool steppersEnabled;
    bool focusUpdated;
    bool tipUpdated;
    bool tiltUpdated;
    int32_t A_cmdSteps;
    int32_t B_cmdSteps;
    int32_t C_cmdSteps;
    bool limitFound_A;
    bool limitFound_B;
    bool limitFound_C;
    double homingSpeedStepsPerSec;

    static void limitSwitch_A_ISR();
    static void limitSwitch_B_ISR();
    static void limitSwitch_C_ISR();

    typedef enum
    {
        IDLE = 0,
        NEW_MOVE_CMD = 1,
        MOVE_IN_PROGRESS = 2,
        MOVE_COMPLETE = 3,
        LIMIT_SW_DETECT = 4,
        HOMING_IS_ACTIVE = 5,
    } MOVE_STATE;
    MOVE_STATE currentMoveState;

    typedef enum
    {
        INITIALIZE,
        HOMING_STEP_1, // Quick move until all endstops are hit
        HOMING_STEP_2, // BOTTOM FOUND
        HOMING_STEP_3, // RETURN TO MIDDLE
    } HOMING_STATE;
    HOMING_STATE currentHomingState;

    volatile bool *moveNotifierFlagPtr;
    volatile bool *homeNotifierFlagPtr;
};

#endif
