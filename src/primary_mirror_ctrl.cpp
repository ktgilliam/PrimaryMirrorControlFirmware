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
#include <iostream>


AccelStepper A(AccelStepper::DRIVER, A_STEP, A_DIR);
AccelStepper B(AccelStepper::DRIVER, B_STEP, B_DIR);
AccelStepper C(AccelStepper::DRIVER, C_STEP, C_DIR);
extern LFAST::TcpCommsService *commsService;

static double velVal        = 0.0;
static double tipVal        = 0.0;
static double tiltVal       = 0.0;
static double focusVal      = 0.0;
static int commthreadID     = 0;
static int ctrlthreadID     = 0;
static int typeVal          = 0;
static int unitVal          = 0;


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Motion Control Functions  //////////////////////////////////////
/////////////////////////////////////////                           //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void hardware_setup()
{
  // Initialize motors + limit switches
  A.setMaxSpeed(800.0); // Steps per second
  A.setAcceleration(100.0); // Steps per second per second
  pinMode(A_LIM, INPUT_PULLUP);

  B.setMaxSpeed(800.0);
  B.setAcceleration(100.0);
  pinMode(B_LIM, INPUT_PULLUP);

  C.setMaxSpeed(800.0);
  C.setAcceleration(100.0);
  pinMode(C_LIM, INPUT_PULLUP);

  //Global stepper enable pin, high to diable drivers
  pinMode(STEP_ENABLE_PIN, OUTPUT);
  digitalWrite(STEP_ENABLE_PIN, HIGH);

}


void set_thread_ID(int commID, int ctrlID)  
{
    if (ctrlID == 0) 
    {
        commthreadID = commID;
        //Serial.print("Comm thread ID set to: ");
        //Serial.println(commthreadID);
    }
    else if (commID == 0) 
    {
        ctrlthreadID = ctrlID;
        //Serial.print("Ctrl thread ID set to: ");
        //Serial.println(ctrlthreadID);
    }
    else {
        //Serial.println("Invalid thread ID."); 
    }
}


int get_thread_ID(bool commID, bool ctrlID) {
    if (ctrlID == 0) 
    {
        return(commthreadID);
    }
    else if (commID == 0) 
    {
        return(ctrlthreadID);
    }
    else {
        //Serial.println("Invalid thread ID requested."); 
        return(0);
    }
}
/*
void connectTerminalInterface(TerminalInterface *_cli)
{
    cli = _cli;
}
*/
// Handshake function to confirm connection
void handshake(unsigned int val) {

    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<unsigned int>("Handshake.", val);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
    //std::string msg = "Connected to client.";
    //mcIf->addDebugMessage(msg);
    return;
}


// Functions to update necessary control variables
void moveType(unsigned int type) 
{
    unsigned int move = 0;    
    LFAST::CommsMessage newMsg;

    if (type == 0) { // absolute
        move = LFAST::ABSOLUTE;
    }
    else if (type == 1) { // relative
        move = LFAST::RELATIVE;
    }
    else {
        newMsg.addKeyValuePair<unsigned int>("Invalid movement type.", 0x0BAD);
        commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
        return;
    }
    moveMirror(LFAST::TYPE, move);
}

void changeVel(double targetVel)
{
    moveMirror(LFAST::VELOCITY, targetVel);
}

void velUnits(unsigned int units)
{
    unsigned int unit = 0;    
    LFAST::CommsMessage newMsg;

    if (units == 0) { //rad / sec
        unit = LFAST::RADSEC;
    }
    else if (units == 1) { // steps / sec
        unit = LFAST::STEPSEC;
    }
    else {
        newMsg.addKeyValuePair<unsigned int>("Invalid velocity units.", 0x0BAD);
        commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
        return;
    }
    moveMirror(LFAST::UNITS, unit);
}
void changeTip(double targetTip)
{
    moveMirror(LFAST::TIP, targetTip);
}
void changeTilt(double targetTilt)
{
    moveMirror(LFAST::TILT, targetTilt);
}
void changeFocus(double targetFocus) 
{
    moveMirror(LFAST::FOCUS, targetFocus);
}
void moveMirror(uint8_t axis, double val) 
{
    static bool velUpdated      = false;
    static bool typeUpdated     = false;
    static bool unitUpdated     = false;
    static bool focusUpdated    = false;
    static bool tipUpdated      = false;
    static bool tiltUpdated     = false;


    if (axis == LFAST::PMC::VELOCITY)
    {
        velVal = val;
        velUpdated = true;
    }
    else if (axis == LFAST::PMC::UNITS)
    {
        unitVal = val;
        unitUpdated = true;
    }
    else if (axis == LFAST::PMC::FOCUS) 
    {
        focusVal = val;
        focusUpdated = true;
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
        typeVal = val;
        typeUpdated = true;
    }
    // tip/tilt adustment control parsing
    if (velUpdated == true && unitUpdated == true && tipUpdated == true && tiltUpdated == true && typeUpdated == true && focusUpdated == false){

        if ((typeVal == LFAST::PMC::ABSOLUTE) && (unitVal == LFAST::PMC::RADSEC)) 
        { 
            Serial.println("moveAbsolute");
            moveAbsolute(velVal, tipVal, tiltVal);
        }
        else if ((typeVal == LFAST::PMC::RELATIVE) && (unitVal == LFAST::PMC::RADSEC)) 
        { 
            Serial.println("moveRelative");
            moveRelative(velVal, tipVal, tiltVal);
        }
        else if ((typeVal == LFAST::PMC::ABSOLUTE) && (unitVal == LFAST::PMC::STEPSEC)) 
        { 
            Serial.println("moveRawAbsolute");
            moveRawAbsolute(velVal, tipVal, tiltVal);
        }
        else if ((typeVal == LFAST::PMC::RELATIVE) && (unitVal == LFAST::PMC::STEPSEC)) 
        { 
            Serial.println("moveRawRelative");
            moveRawRelative(velVal, tipVal, tiltVal);
        }
        velUpdated = false;
        tipUpdated = false;
        tiltUpdated = false;
        typeUpdated = false;
        unitUpdated = false;
    }
    // focus adjustment control parsing
    else if (focusUpdated == true && typeUpdated == true && velUpdated == true && unitUpdated == true) {
        if (typeVal == LFAST::PMC::RELATIVE && unitVal == LFAST::PMC::RADSEC) 
        {
            Serial.println("focusRelative");
            focusRelative(velVal, focusVal);
        }
        else if (typeVal == LFAST::PMC::RELATIVE && unitVal == LFAST::PMC::STEPSEC) 
        {
            Serial.println("focusRelativeRaw");
            focusRelativeRaw(velVal, focusVal);
        }
        else if (typeVal == LFAST::PMC::ABSOLUTE && unitVal == LFAST::PMC::RADSEC) 
        {
            Serial.println("focusAbsolute");
            focusAbsolute(velVal, focusVal);
        }
        else if (typeVal == LFAST::PMC::ABSOLUTE && unitVal == LFAST::PMC::STEPSEC) 
        {
            Serial.println("focusRawAbsolute");
            focusRawAbsolute(velVal, focusVal);
        }
        velUpdated = false;
        unitUpdated = false;
        focusUpdated = false;
        typeUpdated = false;
    }
}


// Move all actuators to home positions at velocity V (steps/sec)
void home(volatile double v) 
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Finding Home", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    A.setSpeed(-v);
    B.setSpeed(-v);
    C.setSpeed(-v);
    // Retract motors until they reach limit switches
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
    // Deploy motors until limit switches deactivate
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
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    A.setCurrentPosition(0);
    B.setCurrentPosition(0);
    C.setCurrentPosition(0);
    save_current_positions();
}


// Move each axis with velocity V to an absolute X,Y position with respect to “home” 
// Velocity input as rad/ sec, converted to steps / sec
void moveAbsolute(double v, double tip, double tilt) 
{
    // Convert v (rad / second) to steps / second 
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel 
    moveRawAbsolute(v, tip, tilt);
}
// Move each axis with velocity V to an absolute X,Y position with respect to “home”
// V, X and Y are vectors of length 3. Velocity is in units of steps per second, X,Y are steps.
// Velocity input as steps / sec
void moveRawAbsolute(double v, double tip, double tilt) 
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Adjusting Tip/tilt Absolute", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

   // Convert x/y inputs to absulute stepper positions relative to zero position (Produces results in mm)
    double Adistance = (281.3*sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0*sin(tilt) - 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));
    double Cdistance = (-0.004 * (60900.0*sin(tilt) + 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));

    // Convert Distance to steps (0.003mm per step??)
    int Asteps = Adistance / (MM_PER_STEP);
    int Bsteps = Bdistance / (MM_PER_STEP);
    int Csteps = Cdistance / (MM_PER_STEP);

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

        if (A.currentPosition() != Asteps) {
            A.runSpeed();
        }
        if (B.currentPosition() != Bsteps) {
            B.runSpeed();
        }
        if (C.currentPosition() != Csteps) {
            C.runSpeed();
        }
    }
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}


// Move each axis with velocity V X,Y units from the current position In the above commands, 
// V, X and Y are vectors of length 3. Velocity is in units of radians per second, X,Y are milliradians.
// Velocity input as rad/ sec, converted to steps / sec
void moveRelative(double v, double tip, double tilt)
{ 
    // Convert v (rad / second) to steps / second 
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel 
    moveRawRelative(v, tip, tilt);                // linear velocity / (1 step / 3 um) = steps / sec 
}
// Move each axis with velocity V X,Y units from the current position to achieve desired tip/tilt(radians) relative position
// Velocity input as steps / sec
void moveRawRelative(double v, double tip, double tilt)
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Adjusting Tip/tilt Relative", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // Convert x/y inputs to absulute stepper positions relative to zero position (Produces results in mm)
    double Adistance = (281.3*sin(tip)) / cos(tip);
    double Bdistance = (0.004 * (60900.0*sin(tilt) - 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));
    double Cdistance = (-0.004 * (60900.0*sin(tilt) + 35160.0*cos(tilt)*sin(tip))) / (cos(tip)*cos(tilt));

    // Convert Distance to steps (MM_PER_STEP mm per step??)
    int Asteps = Adistance / (MM_PER_STEP);
    int Bsteps = Bdistance / (MM_PER_STEP);
    int Csteps = Cdistance / (MM_PER_STEP);

    Serial.println(Asteps);
    Serial.println(Bsteps);
    Serial.println(Csteps);

    A.moveTo(A.currentPosition() + Asteps);
    if(Asteps < 0) 
    {
        A.setSpeed(-v);
    }
    else{
        A.setSpeed(v);
    }
    B.moveTo(B.currentPosition() + Bsteps);
    if(Bsteps < 0) 
    {
        B.setSpeed(-v);
    }
    else{
        B.setSpeed(v);
    }
    C.moveTo(C.currentPosition() + Csteps);
    if(Csteps < 0) 
    {
        C.setSpeed(-v);
    }
    else{
        C.setSpeed(v);
    }

        Serial.print("A: ");
        Serial.println(A.distanceToGo());
        Serial.print("B: ");
        Serial.println(B.distanceToGo());
        Serial.print("C: ");
        Serial.println(C.distanceToGo());

    while ((A.distanceToGo() != 0) || (B.distanceToGo() != 0) || (C.distanceToGo() != 0)) {
        /*
        Serial.print("A: ");
        Serial.println(A.distanceToGo());
        Serial.print("B: ");
        Serial.println(B.distanceToGo());
        Serial.print("C: ");
        Serial.println(C.distanceToGo());*/

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
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}


// Adjust focus position z(um) at velicity v(rad/sec)
void focusRelative(double v, double z) 
{ 
    // Convert v (rad / second) to steps / second 
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel 
    focusRelativeRaw(velVal, focusVal);
}
// Adjust focus position z(um) from current position at velicity v(steps/sec)
void focusRelativeRaw(double v, double z) 
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Adjusting focus Relative.", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // Convert Distance to steps 3um per step??
    // z is in microns??
    int steps = z / 3;

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
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}


// v input in (rad/sec), z input as um
void focusAbsolute(double v, double z) 
{
    // Convert v (rad / second) to steps / second 
    v = (v * MIRROR_RADIUS) / (MICRON_PER_STEP); // angular_vel * mirror radius = linear vel 
    focusRawAbsolute(v, z);
}


// v input in (steps/sec), z input as um
void focusRawAbsolute(double v, double z) 
{
    digitalWrite(STEP_ENABLE_PIN, LOW);
    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Adjusting focus absolute.", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);

    // get z position of actuators in steps (x, y positions are fixed)
    int zA, zB, zC, zf;
    zA = A.currentPosition();
    zB = B.currentPosition();
    zC = C.currentPosition();

    // convert position in steps to mm?
    // MM_PER_STEP microns / step
    zA = (zA * MM_PER_STEP);
    zB = (zB * MM_PER_STEP);
    zC = (zC * MM_PER_STEP); 


    // Calculate z postion of focus given three actuator points.
    // Derived from the position of actuators at any givem moment
    // Equation produces position in mm
    zf = ((((1376420)*(zB - (2*zA) + zC)) / (4129260)) + zA) * 1000; // *1000 to convert mm to um
    // Adjust velocity ( + for up z movement, - for down z movement)
    if (z < zf) 
    {
        v = -abs(v);
    }
    else
    {
        v = abs(v);
    }

    // Difference between desired position and actual position to determine movement amount
    Serial.println(z);
    int move = z - zf;
    // convert micron movement back to steps
    move = move / MICRON_PER_STEP; // microns * (1 step / 3 microns)
    Serial.println(move);

    // Equally adust desired actuator movement given desired focus position  
    A.moveTo(A.currentPosition() + move);
    A.setSpeed(v);
    B.moveTo(B.currentPosition() + move);
    B.setSpeed(v);
    C.moveTo(C.currentPosition() + move);
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
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    save_current_positions();
}


// Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving 
void getStatus(double lst) 
{
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
void getPositions(double lst) 
{
    double A_position, B_position, C_position;

    A_position = A.currentPosition();
    B_position = B.currentPosition();
    C_position = C.currentPosition();

    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<double>("APosition", A_position);
    newMsg.addKeyValuePair<double>("BPosition", B_position);
    newMsg.addKeyValuePair<double>("CPosition", C_position);
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

// Immediately stops all motion
void stop(double lst) 
{
    digitalWrite(STEP_ENABLE_PIN, HIGH);
    int i = 1;
    while ((ctrlthreadID - i) != commthreadID)     // Kill all running control threads
    {
        threads.kill(ctrlthreadID - i); 
        i++;
    }    
    A.stop();
    B.stop();
    C.stop();

    save_current_positions();

    LFAST::CommsMessage newMsg;
    newMsg.addKeyValuePair<std::string>("Stopped", "$OK^");
    commsService->sendMessage(newMsg, LFAST::CommsService::ACTIVE_CONNECTION);
}

// Set the fan speed to a percentage S of full scale
// Fan Pin unknown?
void fanSpeed(unsigned int PWR)
{
    analogWrite(FAN_CONTROL, PWR);
}

void save_current_positions() 
{
    unsigned int eeAddr = 1;
    int Aposition = A.currentPosition();
    int Bposition = B.currentPosition();
    int Cposition = C.currentPosition();

    EEPROM.put(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); //Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); //Move address to the next byte after float 'f'.
    EEPROM.put(eeAddr, Cposition);
}

void load_current_positions() 
{
    unsigned int eeAddr = 1;
    int Aposition = 0;
    int Bposition = 0;
    int Cposition = 0;

    EEPROM.get(eeAddr, Aposition);
    eeAddr += sizeof(Aposition); //Move address to the next byte after float 'f'.
    EEPROM.get(eeAddr, Bposition);
    eeAddr += sizeof(Bposition); //Move address to the next byte after float 'f'.
    EEPROM.get(eeAddr, Cposition);

    A.setCurrentPosition(Aposition);
    B.setCurrentPosition(Bposition);
    C.setCurrentPosition(Cposition);
}










//////////////////////////////////////////////////////////////////////////////////////////
                            /*Code Below is in work*/
//////////////////////////////////////////////////////////////////////////////////////////
// Static or Dynamic? 
void jogMirror(double lst) 
{
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