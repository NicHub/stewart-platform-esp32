/**
 * STEW
 */

#include <iostream>
#include <iomanip>
#include "HexapodKinematics.h"

using namespace std;

int main()
{
    HexapodKinematics hk;       // Stewart platform object.
    double sp_servo[NB_SERVOS]; // Servo setpoints in degrees, between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
    int movOK = -1;

    cout << "\nSTEWART PLATFORM\n\n";
    cout << "COMPILATION DATE AND TIME\n";
    cout << __DATE__ << endl;
    cout << __TIME__ << endl;
    cout << endl;

    const double maxTr = 10;
    const double incTr = 2 * maxTr;
    const double maxRt = 9;
    const double incRt = 2 * maxRt;

    const uint8_t smallWidth = 6;
    const uint8_t largeWidth = 17;
    const uint8_t allWidth = 149;

    cout << fixed << setw(smallWidth) << setfill(' ') << "SWAY" << " ";
    cout << fixed << setw(smallWidth) << setfill(' ') << "SURGE" << " ";
    cout << fixed << setw(smallWidth) << setfill(' ') << "HEAVE" << " ";
    cout << fixed << setw(smallWidth) << setfill(' ') << "PITCH" << " ";
    cout << fixed << setw(smallWidth) << setfill(' ') << "ROLL" << " ";
    cout << fixed << setw(smallWidth) << setfill(' ') << "YAW" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 1" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 2" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 3" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 4" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 5" << " ";
    cout << fixed << setw(largeWidth) << setfill(' ') << "ANGLE 6" << " ";
    cout << endl;
    cout << fixed << setw(allWidth) << setfill('=') << "";
    cout << endl;

    for (double sway = -maxTr; sway <= maxTr; sway += incTr)
    {
        for (double surge = -maxTr; surge <= maxTr; surge += incTr)
        {
            for (double heave = -maxTr; heave <= maxTr; heave += incTr)
            {
                for (double pitch = -maxRt; pitch <= maxRt; pitch += incRt)
                {
                    for (double roll = -maxRt; roll <= maxRt; roll += incRt)
                    {
                        for (double yaw = -maxRt; yaw <= maxRt; yaw += incRt)
                        {
                            movOK = hk.moveTo(sp_servo, sway, surge, heave, pitch, roll, yaw);
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << sway << " ";
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << surge << " ";
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << heave << " ";
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << pitch << " ";
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << roll << " ";
                            cout << fixed << setprecision(1) << setw(smallWidth) << setfill(' ') << yaw << " ";
                            if (movOK == 0)
                            {
                                for (uint8_t id = 0; id < NB_SERVOS; id++)
                                {
                                    cout << fixed << setprecision(6) << setw(largeWidth) << setfill(' ') << sp_servo[id] << " ";
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
