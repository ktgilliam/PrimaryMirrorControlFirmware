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

// Setup functions
void hardware_setup();
void set_thread_ID(int commID, int ctrlID);
int get_thread_ID(bool commID, bool ctrlID);
//void connectTerminalInterface(TerminalInterface* _cli);
void handshake(unsigned int val);

void save_current_positions();
void load_current_positions();

// PMC Command Processing functions
void moveType(unsigned int type);
void changeVel(double targetVel);
void velUnits(unsigned int target);
void changeTip(double targetTip);
void changeTilt(double targetTilt);
void changeFocus(double targetFocus);
void moveMirror(uint8_t axis, double val);

// PM Control functions
void moveAbsolute(double v, double tip, double tilt);
void moveRelative(double v, double tip, double tilt);
void moveRawAbsolute(double v, double tip, double tilt);
void moveRawRelative(double v, double tip, double tilt);
void focusRelative(double v, double z);
void focusRelativeRaw(double v, double z);
void focusAbsolute(double v, double z);
void focusRawAbsolute(double v, double z);
void home(double v);
void getStatus(double lst);
void getPositions(double lst);
void stop(double lst);
void jogMirror(double lst);
void fanSpeed(unsigned int val);

namespace LFAST
{
    enum PMC
    {
        VELOCITY = 0,
        TIP = 1,
        TILT = 2,
        FOCUS = 3,
        TYPE = 4,
        UNITS = 5,
        ABSOLUTE = 6,
        RELATIVE = 7,
        RADSEC = 8, 
        STEPSEC = 9,
    };
}

#endif