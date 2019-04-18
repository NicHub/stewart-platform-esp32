#ifndef __Hexapod_Servo_H__
#define __Hexapod_Servo_H__

#include <main.h>

void setupServos();
void updateServos(int8_t);
void printServoAngles();
void printJointAndServoAxisCoord();

#endif
