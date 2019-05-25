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

#pragma once

// This library can be compiled for ESP32 or for a desktop application.
#ifdef PLATFORMIO
#include <Arduino.h>
#else
#include "../hexapod_app/hexapod_app.h"
#endif

// Choose configuration file.
#define HEXAPOD_CONFIG 1

#if HEXAPOD_CONFIG == 1
#include "Hexapod_Config_1.h"
#elif HEXAPOD_CONFIG == 2
#include "Hexapod_Config_2.h"
#elif HEXAPOD_CONFIG == 3
#include "Hexapod_Config_3.h"
#endif

// `POW` is a lot faster than `pow` defined in cmath.h.
#define POW(base, exp)                                          \
    (exp == 2 ? base * base                                     \
              : exp == 3 ? base * base * base                   \
                         : exp == 4 ? base * base * base * base \
                                    : -1)

// angle_t
typedef struct
{
    double rad;   // Servo angle in radian.
    double deg;   // Servo angle in degrees.
    int us;       // Servo angle in µs (PWM).
    double debug; // Used for debug.
} angle_t;

// calibration_t
typedef struct
{
    double gain;
    double offset;
} calibration_t;

// Platform coordinates.
typedef struct
{
    double hx_x; // Sway, translation along X axis (mm)
    double hx_y; // Surge, translation along Y axis (mm)
    double hx_z; // Heave, translation along Z axis (mm)
    double hx_a; // Pitch, rotation around X axis (rad)
    double hx_b; // Roll, rotation around Y axis (rad)
    double hx_c; // Yaw, rotation around Z axis (rad)
} platform_t;

/**
 *
 */
class Hexapod_Kinematics
{
private:
    // Setpoints (internal states)
    platform_t _coord;

public:
    /*
     * ======== MAIN FUNCTIONS ==========
     */
    Hexapod_Kinematics(){};
    int8_t home(angle_t *servo_angles);
    int8_t calcServoAngles(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo1(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo2(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo3(platform_t coord, angle_t *servo_angles);
    double getHX_X();
    double getHX_Y();
    double getHX_Z();
    double getHX_A();
    double getHX_B();
    double getHX_C();

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
     * Orientations of the servos arms relative to the X axis.
     */
    const double COS_THETA_S[NB_SERVOS] = {
        cos(THETA_S[0]),
        cos(THETA_S[1]),
        cos(THETA_S[2]),
        cos(THETA_S[3]),
        cos(THETA_S[4]),
        cos(THETA_S[5])};

    const double SIN_THETA_S[NB_SERVOS] = {
        sin(THETA_S[0]),
        sin(THETA_S[1]),
        sin(THETA_S[2]),
        sin(THETA_S[3]),
        sin(THETA_S[4]),
        sin(THETA_S[5])};

    // For algorithm 2
    const double angleD[NB_SERVOS] = {
        -THETA_S[0],
        -THETA_S[1],
        -THETA_S[2],
        -THETA_S[3],
        -THETA_S[4],
        -THETA_S[5]};

    const double sinD[NB_SERVOS] = {
        sin(angleD[0]),
        sin(angleD[1]),
        sin(angleD[2]),
        sin(angleD[3]),
        sin(angleD[4]),
        sin(angleD[5])};

    const double cosD[NB_SERVOS] = {
        cos(angleD[0]),
        cos(angleD[1]),
        cos(angleD[2]),
        cos(angleD[3]),
        cos(angleD[4]),
        cos(angleD[5])};

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

    /*
     * The gain in µs/rad (=~ 518 µs/rad).
     */
    const double gain = (SERVO_MAX_US - SERVO_MIN_US) /
                        (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    /*
     * Calibration factors. These values take into account the fact
     * that the odd and even arms are a reflection of each other.
     * The calibration is linear:
     * pulse width (µs) = gain (µs/rad) + offset (µs)
     */
    const calibration_t SERVO_CALIBRATION[NB_SERVOS] = {
        {-gain, SERVO_MAX_US + PW_OFFSET[0]},
        {gain, SERVO_MIN_US + PW_OFFSET[1]},
        {-gain, SERVO_MAX_US + PW_OFFSET[2]},
        {gain, SERVO_MIN_US + PW_OFFSET[3]},
        {-gain, SERVO_MAX_US + PW_OFFSET[4]},
        {gain, SERVO_MIN_US + PW_OFFSET[5]}};

    /*
     * Square of the longest physically possible distance
     * between servo pivot and platform joint (called “d”).
     */
    const double D2MAX = ((ARM_LENGTH + ROD_LENGTH) * (ARM_LENGTH + ROD_LENGTH));

    /*
     * Square of the length of d when the servo arm is perpendicular to d
     * and the rod is the hypotenuse.
     */
    const double D2PERP = (ROD_LENGTH * ROD_LENGTH) - (ARM_LENGTH * ARM_LENGTH);
};
