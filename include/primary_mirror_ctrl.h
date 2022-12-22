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

constexpr double MIRROR_RADIUS_MICRONS = 281880.0;          // Radius of mirror actuator positions in um
constexpr double MICRON_PER_STEP = 3.175 * MICROSTEP_RATIO; // conversion factor of stepper motor steps to vertical movement in um
constexpr double STEPS_PER_MICRON = 1.0 / MICRON_PER_STEP;
constexpr double STEPS_PER_MM = STEPS_PER_MICRON*1000;
constexpr double MM_PER_STEP = 1.0/STEPS_PER_MM;

constexpr double STROKE_MICRON = 12700.0;
constexpr double MAX_STROKE_MICRON = (0.5 * STROKE_MICRON);
constexpr double MIN_STROKE_MICRON = (-0.5 * STROKE_MICRON);
constexpr double STROKE_STEPS = (uint32_t)(STROKE_MICRON / MICRON_PER_STEP); // 4000*MICROSTEP_DIVIDER
constexpr double STROKE_BOTTOM_STEPS = (-0.5*STROKE_STEPS);
constexpr double STROKE_TOP_STEPS = (0.5*STROKE_STEPS);
constexpr double STROKE_BOTTOM_MICRON = STROKE_BOTTOM_STEPS * MICRON_PER_STEP;

#define MIRROR_MATH_COEFFS   \
    {                        \
        281.3, -140.6, 243.6 \
    }

// #define MOTOR_MATH_COEFFS
//     {
//        243.6025537797171781,
//        421.932,
//        205567.4254427672568
//     }

#define MOTOR_MATH_COEFFS                                \
    {                                                    \
        (243.6025537797171781) / (205567.4254427672568), \
            (421.932) / (205567.4254427672568),          \
            1.0                                          \
    }
// PM Control functions
enum PRIMARY_MIRROR_ROWS
{
    BLANK_ROW_0,
    CMD_MODE_ROW,
    TIP_ROW,
    TILT_ROW,
    FOCUS_ROW,
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
    const double c[3] = MOTOR_MATH_COEFFS; // Coefficients calculated based on motor positions

    vectorX<double, 3> getMirrorVector(const int32_t *am, const int32_t *bm, const int32_t *cm) const
    {
        double A = (double)*am;
        double B = (double)*bm;
        double C = (double)*cm;
        vectorX<double, 3> normalVector;
        normalVector[0] = (c[0] * (B + C)) - 2 * c[0] * A;
        normalVector[1] = c[1] * (C - B);
        normalVector[2] = c[2];
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
        auto mirroVector = getMirrorVector(&A_steps, &B_steps, &C_steps);
        mirroVector.normalize();
        auto norm_XZ = mirroVector;
        norm_XZ[1] = 0.0;
        norm_XZ.normalize();
        double tipAngle = std::acos(norm_XZ[2]);
        double tiltAngle = std::acos(mirroVector[2] * std::cos(tipAngle) - mirroVector[0] * std::sin(tipAngle));
        *tip = tipAngle;
        *tilt = tiltAngle;
        // *focus = mean<double>(A_steps, B_steps, C_steps) * MICRON_PER_STEP;
        *focus = 0;
    }
};
class MirrorStates
{
private:
    const double c[3] = MIRROR_MATH_COEFFS; // Coefficients calculated based on  motor positions

public:
    MirrorStates &operator=(MirrorStates const &other)
    {
        noInterrupts();
        TIP_POS_ENG = other.TIP_POS_ENG;
        TILT_POS_ENG = other.TILT_POS_ENG;
        FOCUS_POS_ENG = other.FOCUS_POS_ENG;
        interrupts();
        return *this;
    }

    volatile double TIP_POS_ENG;
    volatile double TILT_POS_ENG;
    volatile double FOCUS_POS_ENG;

    bool getMotorPosnCommands(int32_t *a_steps, int32_t *b_steps, int32_t *c_steps) const
    {

        double tanAlpha = std::tan(TIP_POS_ENG);
        double cosAlpha = std::cos(TIP_POS_ENG);
        double tanBeta = std::tan(TILT_POS_ENG);
        double gamma = this->FOCUS_POS_ENG;

        double a_distance = gamma + (c[0] * tanAlpha);
        double b_distance = gamma + (c[1] * tanAlpha + c[2] * tanBeta / cosAlpha);
        double c_distance = gamma + (c[1] * tanAlpha - c[2] * tanBeta / cosAlpha);

        int32_t a_steps_presat = (int32_t)(a_distance * STEPS_PER_MICRON);
        int32_t b_steps_presat = (int32_t)(b_distance * STEPS_PER_MICRON);
        int32_t c_steps_presat = (int32_t)(c_distance * STEPS_PER_MICRON);
        
        constexpr int32_t stroke_ulim = STROKE_STEPS / 2;
        constexpr int32_t stroke_llim = -1 * stroke_ulim;
        int32_t a_steps_postsat = saturate(a_steps_presat, stroke_llim, stroke_ulim);
        int32_t b_steps_postsat = saturate(b_steps_presat, stroke_llim, stroke_ulim);
        int32_t c_steps_postsat = saturate(c_steps_presat, stroke_llim, stroke_ulim);

        int32_t a_diff = a_steps_presat - a_steps_postsat;
        int32_t b_diff = b_steps_presat - b_steps_postsat;
        int32_t c_diff = c_steps_presat - c_steps_postsat;

        int32_t max_diff = std::max({a_diff, b_diff, c_diff});

        bool saturationFlag = false;
        if (std::abs(max_diff) > 0)
        {
            *a_steps = a_steps_postsat - max_diff;
            *b_steps = b_steps_postsat - max_diff;
            *c_steps = c_steps_postsat - max_diff;
            saturationFlag = true;
        }
        else
        {
            *a_steps = a_steps_presat;
            *b_steps = b_steps_presat;
            *c_steps = c_steps_presat;
        }
        return saturationFlag;
    }
    void resetToZero()
    {
        TIP_POS_ENG = 0.0;
        TILT_POS_ENG = 0.0;
        FOCUS_POS_ENG = 0.0;
    }
        void resetToHomed()
    {
        TIP_POS_ENG = 0.0;
        TILT_POS_ENG = 0.0;
        FOCUS_POS_ENG = STROKE_BOTTOM_MICRON;
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
    void stopNow();
    bool getStatus(uint8_t motor);
    double getStepperPosition(uint8_t motor);

    void saveStepperPositionsToEeprom();
    void resetPositionsInEeprom();
    void loadCurrentPositionsFromEeprom();
    void enableControlInterrupt();
    void setMoveNotifierFlag(volatile bool *flagPtr);
    void setHomingCompleteNotifierFlag(volatile bool *flagPtr);
    bool checkForNewCommand();
    bool isHomingInProgress();

    void limitSwitchHandler(uint16_t axis);
    void enableSteppers(bool doEnable);
    bool isEnabled() { return steppersEnabled; }

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
        HOMING_STEP_2, // Short pause
        HOMING_STEP_3, // Slow Move forward until endstops are cleared
        HOMING_STEP_4, // Shorter pause
        HOMING_STEP_5  // Very slow move backwards until endstops are hit again
    } HOMING_STATE;
    HOMING_STATE currentHomingState;

    volatile bool *moveNotifierFlagPtr;
    volatile bool *homeNotifierFlagPtr;
};

#endif
