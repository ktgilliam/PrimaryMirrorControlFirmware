#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#ifndef PRIMARY_MIRROR_GLOBAL_H
#define PRIMARY_MIRROR_GLOBAL_H


#define TEENSY_4_1

#define X_STEP 2
#define X_DIR  5
#define X_LIM  9

#define Y_STEP 3
#define Y_DIR  6
#define Y_LIM 10

#define Z_STEP 4
#define Z_DIR  7
#define Z_LIM 11

#define ENABLE_PIN 8 

#define SW 37
#define VRx 38
#define VRy 39

//Determine Network values
#define MAC     { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
//#define IP      0,0,0,0
#define GATEWAY 0,0,0,0
#define SUBNET  0,0,0,0












#endif