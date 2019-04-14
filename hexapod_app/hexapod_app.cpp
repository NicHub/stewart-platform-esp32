/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Based on
 * 6dof-stewduino
 * Copyright (C) 2018  Philippe Desrosiers
 * https://github.com/xoxota99/stewy
 *
 * Derived from the work of Daniel Waters
 * https://www.youtube.com/watch?v=1jrP3_1ML9M
 *
 * Modified by Nicolas Jeanmonod
 * ouilogique.com
 * March 2019
 *
 *
 * Copyright (C) 2019  Nicolas Jeanmonod
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
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
     servo_t servo_angles[NB_SERVOS];
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
                                   movOK = hk.calcServoAngles(servo_angles, {sway, surge, heave, pitch, roll, yaw});
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
                                             angle_file << fixed << setprecision(6) << setw(LARGE_WIDTH) << setfill(' ') << degrees(servo_angles[id].rad);
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
