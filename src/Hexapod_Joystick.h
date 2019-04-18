#ifndef __HEXAPOD_JOYSTICK_H__
#define __HEXAPOD_JOYSTICK_H__

#include <main.h>
#include <ouilogique_Joystick.h>

class Hexapod_Joystick : public ouilogique_Joystick
{
  public:
    Hexapod_Joystick(uint8_t pinX, uint8_t pinY, uint8_t pinZ);
    void setupJoystick();
    void joystickControl();
};

#endif
