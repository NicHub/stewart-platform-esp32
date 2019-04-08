/**
 * serial.h
 *
 * ouilogique.com
 * april 2019
 *
 */

#ifndef serial_h
#define serial_h

#include <main.h>
#include <HexapodKinematics.h>

void setupSerial();

bool serialRead(String *message);

#endif // serial_h
