#ifndef __OUILOGIQUE_JOYSTICK_h
#define __OUILOGIQUE_JOYSTICK_h

#include <Arduino.h>

class ouilogique_Joystick
{
  public:
    ouilogique_Joystick(uint8_t, uint8_t, uint8_t);
    int16_t getRawX();
    int16_t getRawY();
    int16_t getX();
    int16_t getY();
    bool getZ();
    void calibrate();
    int16_t getRawValueMidX();
    int16_t getRawValueMidY();

  private:
    uint8_t pinX;
    uint8_t pinY;
    uint8_t pinZ;
    int16_t ANALOG_READ_MIN = 0;
    int16_t ANALOG_READ_MAX = 4095; // 12 bits
    int16_t OUT_MIN = -40;
    int16_t OUT_MID = 0;
    int16_t OUT_MAX = 40;
    int16_t RAWVALUEMINX = ANALOG_READ_MIN;
    int16_t RAWVALUEMAXX = ANALOG_READ_MAX;
    int16_t RAWVALUEMIDX = (RAWVALUEMAXX - RAWVALUEMINX) / 2;
    int16_t RAWVALUEMINY = ANALOG_READ_MIN;
    int16_t RAWVALUEMAXY = ANALOG_READ_MAX;
    int16_t RAWVALUEMIDY = (RAWVALUEMAXY - RAWVALUEMINY) / 2;
    int16_t DEADBAND = 200;
    int16_t DEADBANDMINX = RAWVALUEMIDX - DEADBAND;
    int16_t DEADBANDMAXX = RAWVALUEMIDX + DEADBAND;
    int16_t DEADBANDMINY = RAWVALUEMIDY - DEADBAND;
    int16_t DEADBANDMAXY = RAWVALUEMIDY + DEADBAND;
};

#endif
