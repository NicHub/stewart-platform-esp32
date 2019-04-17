#ifndef __HEXAPODSERVO_H__
#define __HEXAPODSERVO_H__

#include <main.h>

void setupServos();
void updateServos(int8_t);
void printServoAngles();
void printJointAndServoAxisCoord();

#endif
