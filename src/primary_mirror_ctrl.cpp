#include "primary_mirror_ctrl.h"
#include "primary_mirror_global.h"
#include "math.h"

extern AccelStepper tip;
extern AccelStepper tilt;
extern AccelStepper focus;

void home(int v) {

    tip.setSpeed(-v);
    tilt.setSpeed(-v);
    focus.setSpeed(-v);

    bool print = true;

    while ((digitalRead(X_LIM))) {// || (digitalRead(Y_LIM)) || (digitalRead(Z_LIM))) {

        if (print) {
            Serial.println("Retracting motors.");
            print = false;
        }
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
    print = true;
    while (!(digitalRead(X_LIM))) {// || !(digitalRead(Y_LIM)) || !(digitalRead(Z_LIM))) {

        if (print) {
            Serial.println("Setting 0.");
            print = false;
        }
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

    Serial.println("Positions:");
    Serial.println(tip.currentPosition());
    Serial.println(tilt.currentPosition());
    Serial.println(focus.currentPosition());
}

void moveAbsolute(int v, int x, int y, int z) {

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawAbsolute(v, x, y, z);

}

void moveRelative(int v, int x, int y, int z){ 

    // Convert v (rad / second) to steps / second 
    v = v * (200 / (2 * PI)); // 2pi rad = 200 steps

    moveRawRelative(v, x, y, z);
}


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
        xValue = (xValue / 8) - 148;	
        yValue = (yValue / 8) - 172;	
        mapX = map(xValue, 0, 1023, -512, 512);
        mapY = -map(yValue, 0, 1023, -512, 512);

    /*
        if(!(indx%100)) {
            Serial.print("X: ");
            Serial.println(mapX, DEC);
            Serial.print("Y: ");
            Serial.println(mapY, DEC);
        }
        indx++;
    */
        if ((mapX > 200) || (mapX < -200)) { 
            tip.setSpeed(mapX);
            tip.run();
        }
        if ((mapY > 200) || (mapY < -200)) { 
            tilt.setSpeed(mapY*2);
            tilt.run(); 
        }
        //delay(10);	
    }
}

// Returns the status bits for each axis of motion. Bits are Faulted, Home and Moving 
uint8_t getStatus() {

    tip.isRunning(); // Checks to see if the motor is currently running to a target
                     // true if the speed is not zero or not at the target position
    return(0);
}

// Create output data of 3 motors step positions 
void getPositions() {

    tip.currentPosition();
    tilt.currentPosition();
    focus.currentPosition();
    
}


void stop() {

    tip.stop();
    tilt.stop();
    focus.stop();
}

void fanSpeed(int s);



