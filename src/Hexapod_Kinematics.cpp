/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
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

#include "Hexapod_Kinematics.h"

// Algorithm 1 takes ~279 µs / movement.
// Algorithm 2 takes ~474 µs / movement.
// Algorithm 3 takes ~271 µs / movement.
#if ALGO == 1
#include <Hexapod_KinematicsCalcServoAnglesAlgo1.h>
#elif ALGO == 2
#include <Hexapod_KinematicsCalcServoAnglesAlgo2.h>
#elif ALGO == 3
#include <Hexapod_KinematicsCalcServoAnglesAlgo3.h>
#endif

/**
 *
 */
double Hexapod_Kinematics::mapDouble(double x,
                                     double in_min, double in_max,
                                     double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * HOME position. No translation, no rotation.
 */
int8_t Hexapod_Kinematics::home(angle_t *servo_angles)
{
    return calcServoAngles({0, 0, 0, 0, 0, 0}, servo_angles);
}

double Hexapod_Kinematics::getHX_X() { return _coord.hx_x; }
double Hexapod_Kinematics::getHX_Y() { return _coord.hx_y; }
double Hexapod_Kinematics::getHX_Z() { return _coord.hx_z; }
double Hexapod_Kinematics::getHX_A() { return _coord.hx_a; }
double Hexapod_Kinematics::getHX_B() { return _coord.hx_b; }
double Hexapod_Kinematics::getHX_C() { return _coord.hx_c; }
