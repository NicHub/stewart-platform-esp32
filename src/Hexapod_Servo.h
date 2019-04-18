#ifndef __HEXAPODSERVO_H__
#define __HEXAPODSERVO_H__

#include <main.h>

class Hexapod_Servo : public Hexapod_Kinematics
{
  private:
    int8_t _state = 0;

  public:
    Hexapod_Servo();
    void test();
    void setupServo();
    void updateServos(int8_t);
    void printServoAngles();
    void printJointAndServoAxisCoord();
};

#endif
