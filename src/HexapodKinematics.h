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

#ifndef __HEXAPODKINEMATICS_H__
#define __HEXAPODKINEMATICS_H__
#include "HexapodKinematicsConfig.h"

#define NB_SERVOS 6

/**
 *
 */
class HexapodKinematics
{

  private:
    // Setpoints (internal state)
    int _sp_sway = 0,  // sway (x) in mm
        _sp_surge = 0, // surge (y) in mm
        _sp_heave = 0; // heave (z) in mm

    float _sp_pitch = 0, // pitch (x) in radians
        _sp_roll = 0,    // roll (y) in radians
        _sp_yaw = 0;     // yaw (z) in radians

  public:
    int8_t home(float *servoValues);
    int8_t moveTo(float *servoValues, int sway, int surge, int heave, float pitch, float roll, float yaw);
    int8_t moveTo(float *servoValues, float pitch, float roll);

    int getSway();
    int getSurge();
    int getHeave();

    float getPitch();
    float getRoll();
    float getYaw();
};

#endif //__HEXAPODKINEMATICS_H__
