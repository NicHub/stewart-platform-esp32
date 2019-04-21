#ifndef __HEXAPOD_SERVO_H__
#define __HEXAPOD_SERVO_H__

#include <main.h>

class Hexapod_Servo : public Hexapod_Kinematics
{
  private:
    Servo servos[NB_SERVOS]; // Array of servo objects.

  public:
    Hexapod_Servo();
    void setupServo();
    void updateServos(int8_t movOK, unsigned long safetyWait_ms = 25UL);
    void printServoAngles();
    void printJointAndServoAxisCoord();
};

#endif
