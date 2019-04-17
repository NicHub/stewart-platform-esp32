#ifndef __HEXAPODSERIAL_H__
#define __HEXAPODSERIAL_H__

#include <main.h>

void setupSerial();
bool serialRead(String *message);
void serialControl();

#endif
