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

/*
 * ======== SERVO SETTINGS ==========
 */

#define NB_SERVOS 6

/*
 * Servo calibration values.
 * These values should not be used to restrict servo movements.
 * Use MIN/MAX COORDINATES below for that.
 */
const double SERVO_MIN_ANGLE = radians(0);
const double SERVO_MAX_ANGLE = radians(155);
const double SERVO_MID_ANGLE = (SERVO_MIN_ANGLE + SERVO_MAX_ANGLE) / 2;
const double SERVO_MIN_US = 700;
const double SERVO_MAX_US = 2100;

/*
 * Offset values in µs to compensate for arm angle errors.
 */
const int PW_OFFSET[] = {
    -50,
    50,
    50,
    0,
    -50,
    0};

/*
 * Pin numbers for each servo signal.
 */
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
 * Orientations of the servos arms relative to the X axis.
 */
const double THETA_S[NB_SERVOS] = {
    radians(-60),
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120)};

/*
 * MIN/MAX COORDINATES
 * NOTE: The actual min and max for each DOF are interdependent. eg:
 * If the platform is pitched by some amount, the roll min/max will be physically
 * different than what's defined here. These are just the absolute maximums under
 * ideal conditions (eg: max for roll when pitch is zero).
 */
const double HX_X_MIN = -30.0; // -37
const double HX_X_MAX = 30.0;  // 38
const double HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2;
const double HX_X_BAND = HX_X_MAX - HX_X_MIN;

const double HX_Y_MIN = -30.0; // -42
const double HX_Y_MAX = 30.0;  // 38
const double HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2;
const double HX_Y_BAND = HX_Y_MAX - HX_Y_MIN;

const double HX_Z_MIN = -12.0; // -21
const double HX_Z_MAX = 12.0;  // 16
const double HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2;
const double HX_Z_BAND = HX_Z_MAX - HX_Z_MIN;

const double HX_A_MIN = radians(-15.0); // radians(-27)
const double HX_A_MAX = radians(15.0);  // radians(19)
const double HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2;
const double HX_A_BAND = HX_A_MAX - HX_A_MIN;

const double HX_B_MIN = radians(-15.0); // radians(-19)
const double HX_B_MAX = radians(15.0);  // radians(19)
const double HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2;
const double HX_B_BAND = HX_B_MAX - HX_B_MIN;

const double HX_C_MIN = radians(-40.0); // radians(-43)
const double HX_C_MAX = radians(40.0);  // radians(43)
const double HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2;
const double HX_C_BAND = HX_C_MAX - HX_C_MIN;

const double THETA_P = radians(48.0); // Platform joint angle (radians) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes.
const double THETA_B = radians(25.0); // Base Servo pinion angle (radians) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes.
const double P_RAD = 99.55 / 2;       // Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
const double B_RAD = 153.99 / 2;      // Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
const double ARM_LENGTH = 20.0;       // Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
const double ROD_LENGTH = 140.0;      // Push rod length (mm). Distance between pushrod ball joints (servo to platform).
const double Z_HOME = 137.3501805;    // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints. Must Be fine tuned manualy or computed with a numerical solver.
