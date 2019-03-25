#ifndef __OUILOGIQUE_JOYSTICK_h
#define __OUILOGIQUE_JOYSTICK_h

#include <Arduino.h>

#define ANALOG_READ_MIN 0
#define ANALOG_READ_MAX 4095
#define OUT_MIN -40
#define OUT_MID 0
#define OUT_MAX 40
#define RAWVALUEMIDX 1703
#define RAWVALUEMINX ANALOG_READ_MIN
#define RAWVALUEMAXX ANALOG_READ_MAX
#define RAWVALUEMIDY 1686
#define RAWVALUEMINY ANALOG_READ_MIN
#define RAWVALUEMAXY ANALOG_READ_MAX
#define DEADBAND 100
#define DEADBANDMINX RAWVALUEMIDX - DEADBAND
#define DEADBANDMAXX RAWVALUEMIDX + DEADBAND
#define DEADBANDMINY RAWVALUEMIDY - DEADBAND
#define DEADBANDMAXY RAWVALUEMIDY + DEADBAND

class ouilogique_Joystick
{
  public:
    ouilogique_Joystick(uint8_t, uint8_t, uint8_t);
    int16_t getRawX();
    int16_t getRawY();
    int16_t getX();
    int16_t getY();
    bool getZ();

  private:
    uint8_t pinX;
    uint8_t pinY;
    uint8_t pinZ;
};

#endif
