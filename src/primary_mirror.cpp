#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "primary_mirror_global.h"

/*
Features: 
Manual Control: Joystick, Arrow Keys
*/

// Figure out appropriate pin outs
//AccelStepper name(mode, step_pin, direction_pin)
AccelStepper tip(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper tilt(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper focus(AccelStepper::DRIVER, Z_STEP, Z_DIR);

void setup() {
  Serial.begin(9600);
  delay(3000);

  Serial.println("Setup Start.");

  tip.setEnablePin(ENABLE_PIN);
  tip.setPinsInverted(false, false, true);

  tip.setMaxSpeed(200.0); // Steps per second
  tip.setAcceleration(50.0); // Steps per second per second

  tilt.setMaxSpeed(200.0);
  tilt.setAcceleration(100.0);

  focus.setMaxSpeed(200.0);
  focus.setAcceleration(100.0);

  // Set enable pin, diable drivers
 // pinMode(ENABLE_PIN, OUTPUT);
  //digitalWrite(ENABLE_PIN, LOW);

  Serial.println("Setup complete.");

}





void loop() {

 //Serial.println("running motor.");

  //digitalWrite(ENABLE_PIN, HIGH);

  tip.runSpeed();
    
}
