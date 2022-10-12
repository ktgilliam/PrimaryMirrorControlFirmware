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

void moveAbsolute(int v, int x, int y, int z);
void moveRelative(int v, int x, int y, int z);
void moveRawAbsolute(int v, int x, int y, int z);
void moveRawRelative(int v, int x, int y, int z);

void home(int v);
void fanSpeed(int s);
uint8_t getStatus();
void getPositions();
void stop();

void parse_command();

void jogMirror();

#endif