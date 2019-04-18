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

#ifndef __HEXAPOD_KINEMATICS_H__
#define __HEXAPOD_KINEMATICS_H__

// This library can be compiled for ESP32 or for a desktop application.
#ifdef PLATFORMIO
#include <Arduino.h>
#else
#include "../hexapod_app/hexapod_app.h"
#endif

// Choose configuration file.
#define HEXAPOD_CONFIG 1

#if HEXAPOD_CONFIG == 1
#include "Hexapod_KinematicsConfig_1.h"
#elif HEXAPOD_CONFIG == 2
#include "Hexapod_KinematicsConfig_2.h"
#elif HEXAPOD_CONFIG == 3
#include "Hexapod_KinematicsConfig_3.h"
#endif

// angle_t
typedef struct
{
    double rad;   // Servo angle in radian.
    int pw;       // Servo pulse width in µs.
    double debug; // Used for debug.
} angle_t;

// Platform coordinates.
typedef struct
{
    double sway;  // Translation along X axis (mm)
    double surge; // Translation along Y axis (mm)
    double heave; // Translation along Z axis (mm)
    double pitch; // Rotation around X axis (rad)
    double roll;  // Rotation around Y axis (rad)
    double yaw;   // Rotation around Z axis (rad)
} platform_t;

/**
 *
 */
class Hexapod_Kinematics
{
  private:
    // Setpoints (internal states)
    double
        _sp_sway = 0,  // Translation along X axis (mm)
        _sp_surge = 0, // Translation along Y axis (mm)
        _sp_heave = 0, // Translation along Z axis (mm)
        _sp_pitch = 0, // Rotation around X axis (rad)
        _sp_roll = 0,  // Rotation around Y axis (rad)
        _sp_yaw = 0;   // Rotation around Z axis (rad)

  public:
    /*
     * ======== MAIN FUNCTIONS ==========
     */
    Hexapod_Kinematics(){};
    int8_t home(angle_t *servo_angles);
    int8_t calcServoAngles(platform_t coord, angle_t *servo_angles);
    double getSway();
    double getSurge();
    double getHeave();
    double getPitch();
    double getRoll();
    double getYaw();

    /*
     * ======== HELPER FUNCTION ==========
     */
    double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

    /*
     * ======== PRECALCULATED GEOMETRY ==========
     */

    /*
     * There are three axes of symmetry (AXIS1, AXIS2, AXIS3). Looking down on the
     * platform from above (along the Y axis), with 0 degrees being the X-positive line, and traveling
     * in a CC direction, these are at 30 degrees, 120 degrees, and 240 degrees. All
     * the polar coordinates of pivot points, servo centers, etc. are calculated based on
     * an axis, and an offset angle (positive or negative theta) from the axis.
     *
     * NOTE: We make an assumption of mirror symmetry for AXIS3 along the Y axis.
     * That is, AXIS1 is at (e.g.) 30 degrees, and AXIS3 will be at 120 degrees
     * We account for this by negating the value of x-coordinates generated based
     * on this axis later on. This is potentially messy, and should maybe be refactored.
     */
    const double AXIS1 = PI / 6;  //  30 degrees.
    const double AXIS2 = -PI / 2; // -90 degrees.
    const double AXIS3 = AXIS1;

    /*
     * Absolute angle that the servo arm plane of rotation is at, from the world-X axis.
     */
    const double THETA_S[NB_SERVOS] = {
        radians(-60),
        radians(120),
        radians(180),
        radians(0),
        radians(60),
        radians(-120)};

    /*
     * XY cartesian coordinates of the platform joints, based on the polar
     * coordinates (platform radius P_RAD, radial axis AXIS[1|2|3], and offset THETA_P.
     * These coordinates are in the plane of the platform itself.
     */
    const double P_COORDS[NB_SERVOS][2] = {
        {P_RAD * cos(AXIS1 + THETA_P), P_RAD *sin(AXIS1 + THETA_P)},
        {P_RAD * cos(AXIS1 - THETA_P), P_RAD *sin(AXIS1 - THETA_P)},
        {P_RAD * cos(AXIS2 + THETA_P), P_RAD *sin(AXIS2 + THETA_P)},
        {-P_RAD * cos(AXIS2 + THETA_P), P_RAD *sin(AXIS2 + THETA_P)},
        {-P_RAD * cos(AXIS3 - THETA_P), P_RAD *sin(AXIS3 - THETA_P)},
        {-P_RAD * cos(AXIS3 + THETA_P), P_RAD *sin(AXIS3 + THETA_P)}};

    /*
     * XY cartesian coordinates of the servo centers, based on the polar
     * coordinates (base radius B_RAD, radial axis AXIS[1|2|3], and offset THETA_B.
     * These coordinates are in the plane of the base itself.
     */
    const double B_COORDS[NB_SERVOS][2] = {
        {B_RAD * cos(AXIS1 + THETA_B), B_RAD *sin(AXIS1 + THETA_B)},
        {B_RAD * cos(AXIS1 - THETA_B), B_RAD *sin(AXIS1 - THETA_B)},
        {B_RAD * cos(AXIS2 + THETA_B), B_RAD *sin(AXIS2 + THETA_B)},
        {-B_RAD * cos(AXIS2 + THETA_B), B_RAD *sin(AXIS2 + THETA_B)},
        {-B_RAD * cos(AXIS3 - THETA_B), B_RAD *sin(AXIS3 - THETA_B)},
        {-B_RAD * cos(AXIS3 + THETA_B), B_RAD *sin(AXIS3 + THETA_B)}};
};

#endif //__HEXAPOD_KINEMATICS_H__
