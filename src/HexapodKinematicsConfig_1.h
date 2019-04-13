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

#ifndef __HEXAPODKINEMATICSCONFIG_H__
#define __HEXAPODKINEMATICSCONFIG_H__

#ifndef PLATFORMIO
#include "../hexapod_app/hexapod_app.h"
#endif

/*
 * ======== SERVO SETTINGS ==========
 */

#define NB_SERVOS 6

// Which servos are reversed. 1 = reversed, 0 = normal.
// !! First servo is of another type, that’s why it is set to 0 !!
const uint8_t SERVO_REVERSE[6] = {0, 0, 1, 0, 1, 0};

const double SERVO_MIN_ANGLE = 0.0;   // These values don’t seem to be taken into account correctly.
const double SERVO_MAX_ANGLE = 180.0; // These values don’t seem to be taken into account correctly.
const double SERVO_MID_ANGLE = (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2;

const int SERVO_MIN_US = 600;  // default  500 (library 4744/ESP32Servo)
const int SERVO_MAX_US = 2300; // default 2500 (library 4744/ESP32Servo)
const int SERVO_MID_US = (SERVO_MIN_US + SERVO_MAX_US) / 2;

// Trim values, in microseconds, AFTER reversing
const int SERVO_TRIM[] = {
    0,
    20,
    0,
    135,
    0,
    120};

// Pin numbers for each servo signal.
const int SERVO_PINS[] = {
    13,
    15,
    27,
    14,
    33,
    25};


/*
 * ======== GEOMETRY SETTINGS ==========
 */

/*
  NOTE: The actual min and max for each DOF are interdependent. eg:
  If the platform is pitched by some amount, the roll min/max will be physically
  different than what's defined here. These are just the absolute maximums under
  ideal conditions (eg: max for roll when pitch is zero).
*/

const double MIN_SWAY = -40;
const double MAX_SWAY = 40;
const double SWAY_BAND = MAX_SWAY - MIN_SWAY;

const double MIN_SURGE = -40;
const double MAX_SURGE = 40;
const double SURGE_BAND = MAX_SURGE - MIN_SURGE;

const double MIN_HEAVE = -15;
const double MAX_HEAVE = 20;
const double HEAVE_BAND = MAX_HEAVE - MIN_HEAVE;

const double MIN_PITCH = -19;
const double MAX_PITCH = 19;
const double PITCH_BAND = MAX_PITCH - MIN_PITCH;

const double MIN_ROLL = -19;
const double MAX_ROLL = 19;
const double ROLL_BAND = MAX_ROLL - MIN_ROLL;

const double MIN_YAW = -27;
const double MAX_YAW = 27;
const double YAW_BAND = MAX_YAW - MIN_YAW;

const double THETA_P = radians(45.25); // Platform joint angle (radians) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes.
const double THETA_B = radians(24.5);  // Base Servo pinion angle (radians) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes.
const double P_RAD = 50;               // Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
const double B_RAD = 80.2;             // Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
const double ARM_LENGTH = 25.0;        // Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
const double ROD_LENGTH = 155.0;       // Push rod length (mm). Distance between pushrod ball joints (servo to platform).
const double Z_HOME = 148.0;           // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints.

/*
  Prescalar to the output of the platform IK solution for each servo.
  NOTE: Even with aggro, the solution will never fall outside the range of
  [SERVO_ANGLE_MIN .. SERVO_ANGLE_MAX]
*/
const double AGGRO = 1.5;

#endif // __HEXAPODKINEMATICSCONFIG_H__
