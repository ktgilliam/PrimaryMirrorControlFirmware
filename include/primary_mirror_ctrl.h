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

#include <iostream>
#include <LFAST_Device.h>
#include <TerminalInterface.h>

// Setup functions

void set_thread_ID(int commID, int ctrlID);
int get_thread_ID(bool commID, bool ctrlID);
// void connectTerminalInterface(TerminalInterface* _cli);

// PMC Command Processing functions

// PM Control functions
enum PRIMARY_MIRROR_ROWS
{
    BLANK_ROW_0,
    TIP_ROW,
    TILT_ROW,
    FOCUS_ROW
};
namespace LFAST
{
    void updateControlLoop_ISR();
    void enableControlLoopInterrupt();

    class PrimaryMirrorControl : public LFAST_Device
    {
    public:
        static PrimaryMirrorControl &getMirrorController();
        
        virtual ~PrimaryMirrorControl() {}

        void setupPersistentFields() override;

        void moveMirror();
        void setVelocity(double vel);
        void setControlMode(uint8_t moveType);
        void setVelUnits(uint8_t velUnits);
        // void moveMirror(uint8_t axis, double val);
        void setFanSpeed(unsigned int PWR);
        void setTipTarget(double tgt);
        void setTiltTarget(double tgt);
        void setFocusTarget(double tgt);
        void goHome(volatile double homeSpeed);
        void stopNow();
        bool getStatus(uint8_t motor);
        double getPosition(uint8_t motor);
        void moveAbsolute(double v, double tip, double tilt);
        void moveRelative(double v, double tip, double tilt);
        void moveRawAbsolute(double v, double tip, double tilt);
        void moveRawRelative(double v, double tip, double tilt);
        void jogMirror(double lst);

        void focusRelative(double v, double z);
        void focusRelativeRaw(double v, double z);
        void focusAbsolute(double v, double z);
        void focusRawAbsolute(double v, double z);

        void save_current_positions();
        void load_current_positions();

    private:
        PrimaryMirrorControl();
        void hardware_setup();
        double velVal;
        double tipVal;
        double tiltVal;
        double focusVal;

        bool focusUpdated;
        bool tipUpdated;
        bool tiltUpdated;

        int unitVal;

        uint8_t controlMode;
        
    };

    namespace PMC
    {
        enum ControlMode
        {
            STOP = 0,
            RELATIVE = 1,
            ABSOLUTE = 2
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

#endif
