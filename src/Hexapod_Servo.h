#ifndef __HEXAPODSERVO_H__
#define __HEXAPODSERVO_H__

#include <main.h>

class Hexapod_Servo : public Hexapod_Kinematics
{
  private:
    Servo servos[NB_SERVOS]; // Array of servo objects.

  public:
    Hexapod_Servo();
    void setupServo();
    void updateServos(int8_t);
    void printServoAngles();
    void printJointAndServoAxisCoord();
};

#endif
