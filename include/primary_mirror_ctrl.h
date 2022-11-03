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

void handshake(unsigned int val);

void changeVel(double targetVel);
void changeTip(double targetTip);
void changeTilt(double targetTilt);
void moveMirror(uint8_t axis, double val);

void moveAbsolute(double v, double tip, double tilt);
void moveRelative(double v, double tip, double tilt);
void moveRawAbsolute(double v, double tip, double tilt);
void moveRawRelative(double v, double tip, double tilt);
void moveFocus(double v, double z);
void moveFocusRaw(double v, double z);
//void focusAbsolute(double v, double z);
//void focusRawAbsolute(double v, double z);

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
        FOCUS = 4,
        TYPE = 5,
        MOVEABSOLUTE = 5,
        MOVERELATIVE = 6,
        MOVERAWABSOLUTE = 7, 
        MOVERAWRELATIVE = 8,
        MOVEFOCUS = 9,
        MOVEFOCUSRAW = 10
    };
}

#endif