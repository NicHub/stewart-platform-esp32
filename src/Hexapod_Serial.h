#ifndef __Hexapod_Serial_H__
#define __Hexapod_Serial_H__

#include <main.h>

void setupSerial();
bool serialRead(String *message);
void serialControl();

#endif