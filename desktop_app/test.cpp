/**
 * STEW
 */

#include <iostream>
#include <iomanip>
#include <math.h>
#include "test.h"
#include "HexapodKinematics.h"

using namespace std;

int main()
{
    HexapodKinematics hk;      // Stewart platform object.
    double sp_servo[NB_SERVOS]; // Servo setpoints in degrees, between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
    int movOK = -1;

    cout << "\nSTART STEWART PLATFORM\n" << endl;
    cout << "COMPILATION DATE AND TIME:" << endl;
    cout << __DATE__ << endl;
    cout << __TIME__ << endl;
    cout << endl;

    const double maxTr = 10;
    const double incTr = 10;
    const double maxRt = 9;
    const double incRt = 9;

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
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << sway << " ";
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << surge << " ";
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << heave << " ";
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << pitch << " ";
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << roll << " ";
                            cout << fixed << setprecision(1) << setw(5) << setfill(' ') << yaw << " ";
                            if (movOK == 0)
                            {
                                for (uint8_t id = 0; id < NB_SERVOS; id++)
                                {
                                    cout << fixed << setprecision(6) << setw(17) << setfill(' ') << sp_servo[id] << " ";
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
