#ifndef __HEXAPOD_SERIAL_H__
#define __HEXAPOD_SERIAL_H__

#include <main.h>

class Hexapod_Serial
{
  public:
    Hexapod_Serial();
    void setupSerial();
    bool serialRead(String *message);
    void serialControl();
};

#endif
