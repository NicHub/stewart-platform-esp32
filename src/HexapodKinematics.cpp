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

#include "HexapodKinematics.h"

/**
 *
 */
double HexapodKinematics::mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * HOME position. No translation, no rotation.
 */
int8_t HexapodKinematics::home(double *servo_angles)
{
    return calcServoAngles(servo_angles, 0, 0, 0, 0, 0, 0);
}

/**
 * Move to a given sway, surge, heave, pitch, roll and yaw values.
 * We expect pitch, roll and yaw in degrees.
 *
 * Returns = 0 if OK
 * Returns > 0 if Warning
 * Returns < 0 if Error
 *
 * TODO:
 * 1) Understand the “geometry shenanigans” below (k, l, m values).
 * 2) optimize so we don’t run through this loop if we know we’re already at the desired setpoint(s).
 *
 */
int8_t HexapodKinematics::calcServoAngles(double *servo_angles, double sway, double surge, double heave, double pitch, double roll, double yaw)
{
    // Constrain input values.
    sway = constrain(sway, MIN_SWAY, MAX_SWAY);
    surge = constrain(surge, MIN_SURGE, MAX_SURGE);
    heave = constrain(heave, MIN_HEAVE, MAX_HEAVE);
    pitch = constrain(pitch, MIN_PITCH, MAX_PITCH);
    roll = constrain(roll, MIN_ROLL, MAX_ROLL);
    yaw = constrain(yaw, MIN_YAW, MAX_YAW);

    double pivot_x, pivot_y, pivot_z, // Global XYZ coordinates of platform pivot points.
        d2,                           // Distance^2 between servo pivot and platform link.
        k, l, m,                      // Intermediate values.
        servo_rad,                    // Angle (radians) to turn each servo.
        servo_deg;                    // Angle (in degrees) to turn each servo.

    double oldValues[NB_SERVOS];

    // Intermediate values, to avoid recalculating SIN / COS.
    double cr = cos(radians(roll)),
           cp = cos(radians(pitch)),
           cy = cos(radians(yaw)),
           sr = sin(radians(roll)),
           sp = sin(radians(pitch)),
           sy = sin(radians(yaw));

    // Save old values.
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        oldValues[sid] = servo_angles[sid];
    }

    // Assume everything will be OK.
    int8_t movOK = 0;

    // Compute new values.
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        pivot_x = P_COORDS[sid][0] * cr * cy + P_COORDS[sid][1] * (sp * sr * cr - cp * sy) + sway;
        pivot_y = P_COORDS[sid][0] * cr * sy + P_COORDS[sid][1] * (cp * cy + sp * sr * sy) + surge;
        pivot_z = -P_COORDS[sid][0] * sr + P_COORDS[sid][1] * sp * cr + Z_HOME + heave;

        d2 = pow(pivot_x - B_COORDS[sid][0], 2) + pow(pivot_y - B_COORDS[sid][1], 2) + pow(pivot_z, 2);

        // Geometry stuff.
        k = d2 - (pow(ROD_LENGTH, 2) - pow(ARM_LENGTH, 2));
        l = 2 * ARM_LENGTH * pivot_z;
        m = 2 * ARM_LENGTH * (cos(THETA_S[sid]) * (pivot_x - B_COORDS[sid][0]) + sin(THETA_S[sid]) * (pivot_y - B_COORDS[sid][1]));
        servo_rad = asin(k / sqrt(l * l + m * m)) - atan(m / l);

        // Convert radians to an angle between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
        servo_deg = this->mapDouble(degrees(servo_rad), -90.0, 90.0, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Is the required virtual arm length longer than physically possible?
        bool armLengthNOK = sqrt(d2) > (ARM_LENGTH + ROD_LENGTH);

        // Other bad stuff.
        bool otherNOK = abs(k / (sqrt(l * l + m * m))) >= 1;

        // Assign error codes if needed.
        if (armLengthNOK)
        {
            movOK -= 1;
        }
        if (otherNOK)
        {
            movOK -= 3;
        }

        // Don’t compute other servo angles if one is NOK.
        if (movOK < 0)
        {
            break;
        }

        // Limit to SERVO_MAX_ANGLE.
        if (servo_deg > SERVO_MAX_ANGLE)
        {
            movOK += 1;
            servo_deg = SERVO_MAX_ANGLE;
        }
        // Limit to SERVO_MIN_ANGLE.
        else if (servo_deg < SERVO_MIN_ANGLE)
        {
            movOK += 10;
            servo_deg = SERVO_MIN_ANGLE;
        }

        servo_angles[sid] = servo_deg;
    }

    if (movOK >= 0)
    {
        // Update values if everything is OK.
        _sp_sway = sway;
        _sp_surge = surge;
        _sp_heave = heave;
        _sp_pitch = pitch;
        _sp_roll = roll;
        _sp_yaw = yaw;

        // Scale values by aggro.
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            double diff = servo_angles[sid] - SERVO_MID_ANGLE;
            servo_angles[sid] = SERVO_MID_ANGLE + (diff * AGGRO);
            servo_angles[sid] = constrain(servo_angles[sid], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        }
    }
    else
    {
        // Reset back to old values if something went wrong.
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servo_angles[sid] = oldValues[sid];
        }
    }

    return movOK;
}

/*
 * Move to a given pitch / roll angle (in degrees)
 */
int8_t HexapodKinematics::calcServoAngles(double *servo_angles, double pitch, double roll)
{
    return calcServoAngles(servo_angles, _sp_sway, _sp_surge, _sp_heave, pitch, roll, _sp_yaw);
}

double HexapodKinematics::getSway() { return _sp_sway; }
double HexapodKinematics::getSurge() { return _sp_surge; }
double HexapodKinematics::getHeave() { return _sp_heave; }
double HexapodKinematics::getPitch() { return _sp_pitch; }
double HexapodKinematics::getRoll() { return _sp_roll; }
double HexapodKinematics::getYaw() { return _sp_yaw; }
