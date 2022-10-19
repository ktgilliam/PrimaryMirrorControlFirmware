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
#include "primary_mirror_global.h"
#include "math.h"

extern AccelStepper tip;
extern AccelStepper tilt;
extern AccelStepper focus;


// Move all actuators to home positions at velocity V 
void home(int v) {

    tip.setSpeed(-v);
    tilt.setSpeed(-v);
    focus.setSpeed(-v);
    while ((digitalRead(X_LIM)) || (digitalRead(Y_LIM)) || (digitalRead(Z_LIM))) {

        if ((digitalRead(X_LIM))) {
            tip.runSpeed();
        }
        if ((digitalRead(Y_LIM))) {
            tilt.runSpeed();
        }
        if ((digitalRead(Z_LIM))) {
            focus.runSpeed();
        }
    }

    //Time to allow limit switch to settle
    delay(1);
    tip.setSpeed(v);
    tilt.setSpeed(v);
    focus.setSpeed(v);
    while (!(digitalRead(X_LIM)) || !(digitalRead(Y_LIM)) || !(digitalRead(Z_LIM))) {

        if (!(digitalRead(X_LIM))) {
            tip.runSpeed();
        }
        if (!(digitalRead(Y_LIM))) {
            tilt.runSpeed();
        }
        if (!(digitalRead(Z_LIM))) {
            focus.runSpeed();
        }
    }
    tip.setCurrentPosition(0);
    tilt.setCurrentPosition(0);
    focus.setCurrentPosition(0);

    /*Serial.println("Positions:");
    Serial.println(tip.currentPosition());
    Serial.println(tilt.currentPosition());
    Serial.println(focus.currentPosition());*/
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home” 
void moveAbsolute(int v, int x, int y, int z) {

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawAbsolute(v, x, y, z);
}

// Move each axis with velocity V X,Y units from the current position In the above commands, 
// V, X and Y are vectors of length 3. Velocity is in units of radians per second, X,Y are milliradians.
void moveRelative(int v, int x, int y, int z){ 

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawRelative(v, x, y, z);
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
void moveRawAbsolute(int v, int x, int y, int z){

    // Convert x/y inputs to absulute stepper positions relative to zero position
    int tipSteps = x;
    int tiltSteps = y;
    int focusSteps = z;

    tip.moveTo(tipSteps);
    tip.setSpeed(v);
    tilt.moveTo(tiltSteps);
    tilt.setSpeed(v);
    focus.moveTo(focusSteps);
    focus.setSpeed(v);

    while ((tip.distanceToGo() != 0) || (tilt.distanceToGo() != 0) || (focus.distanceToGo() != 0)) {

        if ((tip.distanceToGo() != 0)) {
            tip.runSpeed();
        }
        if (tilt.distanceToGo() != 0) {
            tilt.runSpeed();
        }
        if (focus.distanceToGo() != 0) {
            focus.runSpeed();
        }
    }
}

// Move each axis with velocity V X,Y units from the current position In the above commands, 
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
void moveRawRelative(int v, int x, int y, int z) {

    // Convert x/y inputs to relative stepper steps 
    int tipSteps = x;
    int tiltSteps = y;
    int focusSteps = z;

    tip.setSpeed(v);
    tilt.setSpeed(v);
    focus.setSpeed(v);

    while ((tip.currentPosition() != tipSteps) || (tilt.currentPosition() != tiltSteps) || (focus.currentPosition() != focusSteps)) {

        if ((tip.currentPosition() != tipSteps)) {
            tip.runSpeed();
        }
        if (tilt.currentPosition() != tiltSteps) {
            tilt.runSpeed();
        }
        if (focus.currentPosition() != focusSteps) {
            focus.runSpeed();
        }
    }
}

// Static or Dynamic? 
void jogMirror() {

    while (digitalRead(SW)){

        static int indx=0;
        int xValue = 0 ;
        int yValue = 0 ; 
        int mapX = 0;
        int mapY = 0;

        for(int i = 0; i < 8; i++) {
            xValue = xValue + analogRead(VRx);	
            yValue = yValue + analogRead(VRy);	

        }
        xValue = (xValue / 8) - 127;	
        yValue = (yValue / 8) - 149;	
        mapX = map(xValue, 0, 1023, -512, 512);
        mapY = -map(yValue, 0, 1023, -512, 512);

    
        if(!(indx%1000)) {
            Serial.print("X: ");
            Serial.println(mapX, DEC);
            Serial.print("Y: ");
            Serial.println(mapY, DEC);
        }
        indx++;
    
        if ((mapX > 200) || (mapX < -200)) { 
            tip.setSpeed(mapX);
            tip.run();
        }
        if ((mapY > 200) || (mapY < -200)) { 
            tilt.setSpeed(mapY);
            tilt.run(); 
        }
        //delay(10);	
    }
}

// Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving 
uint8_t getStatus() {

    tip.isRunning();  // Checks to see if the motor is currently running to a target
    tilt.isRunning(); // true if the speed is not zero or not at the target position
    focus.isRunning();
    
    return(0); // Return what? Array of status bits?
}

// Returns 3 step counts
uint8_t getPositions() {

    tip.currentPosition();
    tilt.currentPosition();
    focus.currentPosition();

    return(0); // Return what? Array of step counts?
}

// Immediately stops all motion
void stop() {

    tip.stop();
    tilt.stop();
    focus.stop();
}

// Set the fan speed to a percentage S of full scale
void fanSpeed(int s);



