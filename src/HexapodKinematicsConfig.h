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

#define NB_SERVOS 6

// Which servos are reversed. 1 = reversed, 0 = normal.
// !! First servo is of another type, that’s why it is set to 0 !!
const int SERVO_REVERSE[6] = {0, 0, 1, 0, 1, 0};

#define SERVO_MIN_ANGLE 0   // These values don’t seem to be taken into account correctly.
#define SERVO_MAX_ANGLE 180 // These values don’t seem to be taken into account correctly.
const int SERVO_MID_ANGLE = SERVO_MIN_ANGLE + (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 2;

#define SERVO_MIN_US 600  // default  500 (library 4744/ESP32Servo)
#define SERVO_MAX_US 2300 // default 2500 (library 4744/ESP32Servo)
const int SERVO_MID_US = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) / 2;

const int SERVO_TRIM[] = { // trim values, in microseconds, AFTER reversing
    0,
    20,
    0,
    135,
    0,
    120};

const int SERVO_PINS[] = { // pin numbers for each servo signal.
    13,
    15,
    27,
    14,
    33,
    25};

/*
  NOTE: The actual "min" and "max" for each DOF are interdependent. eg:
  If the platform is pitched by some amount, the roll min/max will be physically
  different than what's defined here. These are just the absolute maximums under
  ideal conditions (eg: max for roll when pitch is zero).
*/

#define MIN_PITCH -20
#define MAX_PITCH 23
const int PITCH_BAND = MAX_PITCH - MIN_PITCH;

#define MIN_ROLL -23
#define MAX_ROLL 20
const int ROLL_BAND = MAX_ROLL - MIN_ROLL;

#define MIN_YAW -69
#define MAX_YAW 69
const int YAW_BAND = MAX_YAW - MIN_YAW;

#define MIN_SWAY -55
#define MAX_SWAY 55
const int SWAY_BAND = MAX_SWAY - MIN_SWAY;

#define MIN_SURGE -70
#define MAX_SURGE 55
const int SURGE_BAND = MAX_SURGE - MIN_SURGE;

#define MIN_HEAVE -22
#define MAX_HEAVE 25
const int HEAVE_BAND = MAX_HEAVE - MIN_HEAVE;

/*
* ======== Platform / Servo Settings ==========
*/

// Geometry of the platform.

#define THETA_P_DEG 45.25                // Platform joint angle (degrees) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes
#define THETA_B_DEG 24.5                 // Base Servo pinion angle (degrees) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes
#define THETA_P (THETA_P_DEG * PI / 180) // Theta P, in radians
#define THETA_B (THETA_B_DEG * PI / 180) // Theta B, in radians
#define P_RAD 50                         // Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
#define B_RAD 80.2                       // Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
#define ARM_LENGTH 25                    // Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
#define ROD_LENGTH 155                   // Push rod length (mm). Distance between pushrod ball joints (servo to platform).
#define Z_HOME 148                       // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints.

/*
  Prescalar to the output of the platform IK solution for each servo.
  NOTE: Even with aggro, the solution will never fall outside the range of
  [SERVO_ANGLE_MIN .. SERVO_ANGLE_MAX]
*/
#define AGGRO 1.5F

#endif // __HEXAPODKINEMATICSCONFIG_H__
