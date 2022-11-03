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

extern AccelStepper A;
extern AccelStepper B;
extern AccelStepper C;
extern LFAST::TcpCommsService *commsService;


void handshake(unsigned int val) {

    LFAST::CommsMessage newMsg;
    if (val == 0xDEAD)
    {
        newMsg.addKeyValuePair<unsigned int>("Handshake", 0xBEEF);
        commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
        std::string msg = "Connected to client.";
        //mcIf->addDebugMessage(msg);
    }
    else
    {
        // TODO: Generate error
    }
    return;
}

// Move all actuators to home positions at velocity V 
void home(double v) {

    A.setSpeed(-v);
    B.setSpeed(-v);
    C.setSpeed(-v);
    while ((digitalRead(A_LIM)) || (digitalRead(B_LIM)) || (digitalRead(C_LIM))) {

        if ((digitalRead(A_LIM))) {
            A.runSpeed();
        }
        if ((digitalRead(B_LIM))) {
            B.runSpeed();
        }
        if ((digitalRead(C_LIM))) {
            C.runSpeed();
        }
    }

    //Time to allow limit switch to settle
    delay(1);
    A.setSpeed(v);
    B.setSpeed(v);
    C.setSpeed(v);
    while (!(digitalRead(A_LIM)) || !(digitalRead(B_LIM)) || !(digitalRead(C_LIM))) {

        if (!(digitalRead(A_LIM))) {
            A.runSpeed();
        }
        if (!(digitalRead(B_LIM))) {
            B.runSpeed();
        }
        if (!(digitalRead(C_LIM))) {
            C.runSpeed();
        }
    }
    A.setCurrentPosition(0);
    B.setCurrentPosition(0);
    C.setCurrentPosition(0);

    //LFAST::CommsMessage newMsg;
    //newMsg.addKeyValuePair<std::string>("FindHome", "$OK^");
    //commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

void moveType(double type) 
{
    moveMirror(LFAST::TYPE, type);
}
void changeVel(double targetVel)
{
    moveMirror(LFAST::VELOCITY, targetVel);
}
void changeTip(double targetTip)
{
    moveMirror(LFAST::TIP, targetTip);
}
void changeTilt(double targetTilt)
{
    moveMirror(LFAST::TILT, targetTilt);
}
void changeFocus(double targetFocus) {

    moveMirror(LFAST::FOCUS, targetFocus);
}


void moveMirror(uint8_t axis, double val) {

    static bool velUpdated = false;
    static bool tipUpdated = false;
    static bool tiltUpdated = false;
    static bool typeUpdated = false;
    static bool focusUpdated = false;
    static double velVal = 0.0;
    static double tipVal = 0.0;
    static double tiltVal = 0.0;
    static int typeVal = 0;

    if (axis == LFAST::PMC::VELOCITY)
    {
        velVal = val;
        velUpdated = true;
    }
    else if (axis == LFAST::PMC::TIP)
    {
        tipVal = val;
        tipUpdated = true;
    }
    else if (axis == LFAST::PMC::TILT)
    {
        tiltVal = val;
        tiltUpdated = true;
    }
    else if (axis == LFAST::PMC::TYPE)
    {
        tipVal = val;
        focusUpdated = true;
    }
    else if (axis == LFAST::PMC::FOCUS) {

    }
    if (velUpdated == true && tipUpdated == true && tiltUpdated == true && typeUpdated == true){
        if (typeVal == LFAST::PMC::MOVEABSOLUTE) 
        { 
            moveAbsolute(velVal, tipVal, tiltVal);
        }
        else if (typeVal == LFAST::PMC::MOVERELATIVE) 
        { 
            moveRelative(velVal, tipVal, tiltVal);
        }
        else if (typeVal == LFAST::PMC::MOVERAWABSOLUTE) 
        { 
            moveRawAbsolute(velVal, tipVal, tiltVal);
        }
        else if (typeVal == LFAST::PMC::MOVERAWRELATIVE) 
        { 
            moveRawRelative(velVal, tipVal, tiltVal);
        }
        velUpdated = false;
        tipUpdated = false;
        tiltUpdated = false;
        typeUpdated = false;
    }
    else if (focusUpdated == true && tipUpdated == true && typeUpdated == true && velUpdated == true) {
        if (typeVal == LFAST::PMC::MOVEFOCUS) 
        { 
            moveFocus(velVal, tipVal);
        }
        else if (typeVal == LFAST::PMC::MOVEFOCUSRAW) 
        { 
            moveFocusRaw(velVal, tipVal);
        }
        velUpdated = false;
        tipUpdated = false;
        focusUpdated = false;
        typeUpdated = false;
    }
}


// Move each axis with velocity V to an absolute X,Y position with respect to “home” 
// Velocity input as rad/ sec, converted to steps / sec
void moveAbsolute(double v, double tip, double tilt) {

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawAbsolute(v, tip, tilt);
}

// Move each axis with velocity V X,Y units from the current position In the above commands, 
// V, X and Y are vectors of length 3. Velocity is in units of radians per second, X,Y are milliradians.
// Velocity input as rad/ sec, converted to steps / sec
void moveRelative(double v, double tip, double tilt){ 

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawRelative(v, tip, tilt);
}

// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void moveRawAbsolute(double v, double tip, double tilt) {

    // Convert x/y inputs to absulute stepper positions relative to zero position
    double Adistance = (281.3*sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0*sin(tilt) - 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));
    double Cdistance = (-0.004 * (60900.0*sin(tilt) + 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));

    // Convert Distance to steps. 3um per step??
    int Asteps = Adistance / (0.000003);
    int Bsteps = Bdistance / (0.000003);
    int Csteps = Cdistance / (0.000003);

    // Set directions based on desired positions relative to current positions of steppers.
    if (A.currentPosition() > Asteps) {
        A.setSpeed(-v);
    }
    else {
        A.setSpeed(v);
    }
    if (B.currentPosition() > Bsteps) {
        B.setSpeed(-v);
    }
    else {
        B.setSpeed(v);
    }
    if (C.currentPosition() > Csteps) {
        C.setSpeed(-v);
    }
    else {
        C.setSpeed(v);
    }

    while ((A.currentPosition() != Asteps) || (B.currentPosition() != Bsteps) || (C.currentPosition() != Csteps)) {

        if ((A.currentPosition() != Asteps)) {
            A.runSpeed();
        }
        if (B.currentPosition() != Bsteps) {
            B.runSpeed();
        }
        if (C.currentPosition() != Csteps) {
            C.runSpeed();
        }
    }
}

// Move each axis with velocity V X,Y units from the current position In the above commands,
// Velocity input as steps / sec
void moveRawRelative(double v, double tip, double tilt){

    // Convert x/y inputs to absulute stepper positions relative to zero position
    double Adistance = (281.3*sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0*sin(tilt) - 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));
    double Cdistance = (-0.004 * (60900.0*sin(tilt) + 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));

    // Convert Distance to steps 3um per step??
    int Asteps = Adistance / (0.000003);
    int Bsteps = Bdistance / (0.000003);
    int Csteps = Cdistance / (0.000003);

    A.moveTo(A.currentPosition() + Asteps);
    A.setSpeed(v);
    B.moveTo(B.currentPosition() + Bsteps);
    B.setSpeed(v);
    C.moveTo(C.currentPosition() + Csteps);
    C.setSpeed(v);

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0)) {

        if (A.distanceToGo() != 0) {
            A.runSpeed();
        }
        if (B.distanceToGo() != 0) {
            B.runSpeed();
        }
        if (C.distanceToGo() != 0) {
            C.runSpeed();
        }
    }
}

void moveFocus(double v, double z) { 

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveFocusRaw(v, z);
}
void moveFocusRaw(double v, double z ) {

    // Convert Distance to steps 3um per step??
    int steps = z / (0.000003);

    if (z < 0) {
        v = -v;
    }
    A.moveTo(A.currentPosition() + steps);
    A.setSpeed(v);
    B.moveTo(B.currentPosition() + steps);
    B.setSpeed(v);
    C.moveTo(C.currentPosition() + steps);
    C.setSpeed(v);

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0)) {

        if (A.distanceToGo() != 0) {
            A.runSpeed();
        }
        if (B.distanceToGo() != 0) {
            B.runSpeed();
        }
        if (C.distanceToGo() != 0) {
            C.runSpeed();
        }
    }
}

/*
void focusAbsolute(double v, double z) { 

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    focusRawAbsolute(v, z);
}
void focusRawAbsolute(double v, double z) {}
*/




// Static or Dynamic? 
void jogMirror(double lst) {

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
        mapX = mapX / (PI * 4);
        mapY = mapY / (PI * 4);


        double Aspeed = (281.3*sin(mapX)) / cos(mapX);
        double Bspeed = (0.004 * (60900.0*sin(mapY) - 35160.0*cos(mapY)*sin(mapX))) / (cos(mapX)*cos(mapY));
        double Cspeed = (-0.004 * (60900.0*sin(mapY) + 35160.0*cos(mapY)*sin(mapX))) / (cos(mapX)*cos(mapY));


    
        if(!(indx%1000)) {
            Serial.print("X: ");
            Serial.println(xValue, DEC);
            Serial.print("Y: ");
            Serial.println(yValue, DEC);
            Serial.print("mapX: ");
            Serial.println(mapX, DEC);
            Serial.print("mapY: ");
            Serial.println(mapY, DEC);
            Serial.print("A: ");
            Serial.println(Aspeed, DEC);
            Serial.print("B: ");
            Serial.println(Bspeed, DEC);
            Serial.print("C: ");
            Serial.println(Cspeed, DEC);
        }

        indx++;
    
        if ((mapX > 200) || (mapX < -200) || (mapY > 200) || (mapY < -200)) { 
            A.setSpeed(Aspeed);
            A.run();
            B.setSpeed(Bspeed);
            B.run(); 
            C.setSpeed(Cspeed);
            C.run(); 
        }
        //delay(10);	
    }
}

// Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving 
void getStatus(double lst) {

    bool A_status, B_status, C_status;
    
    A_status = A.isRunning();  // Checks to see if the motor is currently running to a target
    B_status = B.isRunning(); // true if the speed is not zero or not at the target position
    C_status = C.isRunning();

    LFAST::CommsMessage newMsg;

    newMsg.addKeyValuePair<bool>("ARunning?", A_status);
    newMsg.addKeyValuePair<bool>("BRunning?", B_status);
    newMsg.addKeyValuePair<bool>("CRunning?", C_status);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
    
}

// Returns 3 step counts
void getPositions(double lst) {

    double A_position, B_position, C_position;

    A_position = A.currentPosition();
    B_position = B.currentPosition();
    C_position = C.currentPosition();

    LFAST::CommsMessage newMsg;

    newMsg.addKeyValuePair<bool>("APosition", A_position);
    newMsg.addKeyValuePair<bool>("BPosition", B_position);
    newMsg.addKeyValuePair<bool>("CPosition", C_position);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

}

// Immediately stops all motion
void stop(double lst) {

    A.stop();
    B.stop();
    C.stop();
}

// Set the fan speed to a percentage S of full scale
// Fan Pin unknown?
void fanSpeed(unsigned int PWR){

    analogWrite(FAN_CONTROL, PWR);

}



