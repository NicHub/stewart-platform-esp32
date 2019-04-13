/**
 * STEW
 */

#include <iostream>
#include <iomanip>
#include "HexapodKinematics.h"

using namespace std;

int main()
{
    cout << "\nSTEWART PLATFORM\n\n";
    cout << "COMPILATION DATE AND TIME\n";
    cout << __DATE__ << endl;
    cout << __TIME__ << endl;
    cout << endl;

    HexapodKinematics hk;           // Stewart platform object.
    double servo_angles[NB_SERVOS]; // Servo setpoints in degrees, between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
    int movOK = -1;

    const double MAX_TR = 10;
    const double INC_TR = 2 * MAX_TR;
    const double MAX_RT = 9;
    const double INC_RT = 2 * MAX_RT;

    const uint8_t SMALL_WIDTH = 6;
    const uint8_t LARGE_WIDTH = 17;
    const uint8_t ALL_WIDTH = 149;

    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "SWAY"
         << " ";
    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "SURGE"
         << " ";
    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "HEAVE"
         << " ";
    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "PITCH"
         << " ";
    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "ROLL"
         << " ";
    cout << fixed << setw(SMALL_WIDTH) << setfill(' ') << "YAW"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 1"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 2"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 3"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 4"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 5"
         << " ";
    cout << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 6"
         << " ";
    cout << endl;
    cout << fixed << setw(ALL_WIDTH) << setfill('=') << "";
    cout << endl;

    for (double sway = -MAX_TR; sway <= MAX_TR; sway += INC_TR)
    {
        for (double surge = -MAX_TR; surge <= MAX_TR; surge += INC_TR)
        {
            for (double heave = -MAX_TR; heave <= MAX_TR; heave += INC_TR)
            {
                for (double pitch = -MAX_RT; pitch <= MAX_RT; pitch += INC_RT)
                {
                    for (double roll = -MAX_RT; roll <= MAX_RT; roll += INC_RT)
                    {
                        for (double yaw = -MAX_RT; yaw <= MAX_RT; yaw += INC_RT)
                        {
                            movOK = hk.calcServoAngles(servo_angles, sway, surge, heave, pitch, roll, yaw);
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << sway << " ";
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << surge << " ";
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << heave << " ";
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << pitch << " ";
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << roll << " ";
                            cout << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << yaw << " ";
                            if (movOK == 0)
                            {
                                for (uint8_t id = 0; id < NB_SERVOS; id++)
                                {
                                    cout << fixed << setprecision(6) << setw(LARGE_WIDTH) << setfill(' ') << servo_angles[id] << " ";
                                }
                            }
                            if (movOK > 0)
                            {
                                cout << "MAX/MIN  " << movOK;
                            }

                            cout << endl;
                        }
                    }
                }
            }
        }
    }
    return 0;
}
