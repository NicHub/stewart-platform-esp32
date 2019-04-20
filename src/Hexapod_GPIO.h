#ifndef __HEXAPOD_GPIO_H__
#define __HEXAPOD_GPIO_H__

#include <main.h>

class Hexapod_GPIO
{
  private:
    bool builtInLEDState;

  public:
    void setupGPIO();
    void setBuiltInLED();
    void clearBuiltInLED();
    void clearBuiltInLEDDelayed(unsigned long dt);
    void blinkBuitInLED(uint8_t nb_iter, unsigned long tON, unsigned long tOFF);
};

#endif
