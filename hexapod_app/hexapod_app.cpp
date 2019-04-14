/**
 * STEW
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include "HexapodKinematics.h"

using namespace std;

int main()
{
     ofstream angle_file;

     angle_file.open("angles.txt");

     angle_file << "\nSTEWART PLATFORM\n\n";
     angle_file << "COMPILATION DATE AND TIME\n";
     angle_file << __DATE__ << endl;
     angle_file << __TIME__ << endl;
     angle_file << endl;

     HexapodKinematics hk; // Stewart platform object.
     servo_t servo_val[NB_SERVOS];
     int movOK = -1;

     const double MAX_TR = 10;
     const double INC_TR = 2 * MAX_TR;
     const double MAX_RT = radians(9);
     const double INC_RT = 2 * MAX_RT;

     const uint8_t SMALL_WIDTH = 7;
     const uint8_t LARGE_WIDTH = 17;
     const uint8_t ALL_WIDTH = 144;

     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "SWAY";
     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "SURGE";
     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "HEAVE";
     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "PITCH";
     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "ROLL";
     angle_file << fixed << setw(SMALL_WIDTH) << setfill(' ') << "YAW";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 1";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 2";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 3";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 4";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 5";
     angle_file << fixed << setw(LARGE_WIDTH) << setfill(' ') << "ANGLE 6";

     angle_file << endl;
     angle_file << fixed << setw(ALL_WIDTH) << setfill('=') << "";
     angle_file << endl;

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
                                   movOK = hk.calcServoAngles(servo_val, sway, surge, heave, pitch, roll, yaw);
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << sway;
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << surge;
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << heave;
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << degrees(pitch);
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << degrees(roll);
                                   angle_file << fixed << setprecision(1) << setw(SMALL_WIDTH) << setfill(' ') << degrees(yaw);
                                   if (movOK == 0)
                                   {
                                        for (uint8_t id = 0; id < NB_SERVOS; id++)
                                        {
                                             angle_file << fixed << setprecision(6) << setw(LARGE_WIDTH) << setfill(' ') << degrees(servo_val[id].rad);
                                        }
                                   }
                                   if (movOK > 0)
                                   {
                                        angle_file << " movOK = " << movOK;
                                   }

                                   angle_file << endl;
                              }
                         }
                    }
               }
          }
     }
     angle_file.close();
     return 0;
}
