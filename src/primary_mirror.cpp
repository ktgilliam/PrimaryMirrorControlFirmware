#include "primary_mirror_global.h"
#include "primary_mirror_ctrl.h"
#include "primary_mirror_network.h"

// Parsing of JSON style command done in network file, for now.

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

  tip.setMaxSpeed(1200.0); // Steps per second
  tip.setAcceleration(100.0); // Steps per second per second
  pinMode(X_LIM, INPUT_PULLUP);

  tilt.setMaxSpeed(1200.0);
  tilt.setAcceleration(200.0);
  pinMode(Y_LIM, INPUT_PULLUP);

  focus.setMaxSpeed(200.0);
  focus.setAcceleration(100.0);
  pinMode(Z_LIM, INPUT_PULLUP);

  //Set enable pin, diable drivers
  pinMode(ENABLE_PIN, OUTPUT);
  //Set high to disable drivers 
  digitalWrite(ENABLE_PIN, HIGH);

  // Joystick Jogging Enable
  //pinMode(VRx, INPUT);
  //pinMode(VRy, INPUT);
  pinMode(SW, INPUT_PULLUP); 

  Serial.println("Setup complete.");
}





void loop() {

  static int indx=0;

if (indx == 0) {
  jogMirror();
  //home(200);
}

/*
  if (tilt.distanceToGo() == 0)
    tilt.moveTo(-tilt.currentPosition());

  tilt.run();


  if(!(indx%10000))
  {
    Serial.println(tilt.currentPosition());
  }
*/
  indx++;  
}
