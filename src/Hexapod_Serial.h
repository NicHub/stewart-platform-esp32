#ifndef __HEXAPOD_SERIAL_H__
#define __HEXAPOD_SERIAL_H__

#include <main.h>

class Hexapod_Serial
{
  public:
    Hexapod_Serial();
    void setupSerial();
#if ENABLE_SERIAL_READ
    bool serialRead(String *message);
    void serialControl();
#endif
};

#endif
