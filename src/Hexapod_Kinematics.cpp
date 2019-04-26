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

/**
 * Calculate the servo angles in radians, degrees and in pulse width (PWM)
 * given the desired target platform coordinates.
 *
 * @param coord : the desired target platform coordinates.
 * A struct containing X (sway), Y (surge), Z (heave) in mm
 * and A (pitch), B (roll) and C (yaw) in radians.
 *
 * @param servo_angles : pointer to an array of struct containing
 * the calculated servos angles in radians, degrees and in pulse width (PWM).
 *
 * @return
 * Returns = 0 if OK
 * Returns < 0 if Error
 *
 */
int8_t Hexapod_Kinematics::calcServoAngles(platform_t coord, angle_t *servo_angles)
{
    double pivot_x, pivot_y, pivot_z, // Global XYZ coordinates of platform pivot points.
        d2,                           // Distance^2 between servo pivot and platform link.
        k, m, n,                      // Intermediate values.
        servo_rad;                    // Angle (radians) to turn each servo.

    // Intermediate values, to avoid recalculating sin and cos.
    // (3 µs).
    double cosA = cos(coord.hx_a),
           cosB = cos(coord.hx_b),
           cosC = cos(coord.hx_c),
           sinA = sin(coord.hx_a),
           sinB = sin(coord.hx_b),
           sinC = sin(coord.hx_c);

    // Assume everything will be OK.
    int8_t movOK = 0;

    // Compute new angle values.
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        // Coordinates of platform joints (pivots) after movement.
        // Minus B_COORDS.x for pivot_x.
        // Minus B_COORDS.y for pivot_y.
        // (~7 µs)
        pivot_x = P_COORDS[sid][0] * cosB * cosC +
                  P_COORDS[sid][1] * (sinA * sinB * cosB - cosA * sinC) +
                  coord.hx_x -
                  B_COORDS[sid][0];
        pivot_y = P_COORDS[sid][0] * cosB * sinC +
                  P_COORDS[sid][1] * (cosA * cosC + sinA * sinB * sinC) +
                  coord.hx_y -
                  B_COORDS[sid][1];
        pivot_z = -P_COORDS[sid][0] * sinB +
                  P_COORDS[sid][1] * sinA * cosB +
                  Z_HOME +
                  coord.hx_z;

        // Square of the virtual arm length, i.e. the distance
        // between the base joint and the platform joint.
        // (~3 µs)
        d2 = (pivot_x * pivot_x) +
             (pivot_y * pivot_y) +
             (pivot_z * pivot_z);

        // Test if the required virtual arm length is longer than physically possible.
        // Abort computation of remaining angles if this one is not OK.
        // (~1 µs)
        if (d2 > ((ARM_LENGTH + ROD_LENGTH) * (ARM_LENGTH + ROD_LENGTH)))
        {
            movOK = -1;
            break;
        }

        // Calculation of intermediate values.
        // (~1 µs)
        k = d2 -
            (ROD_LENGTH * ROD_LENGTH) +
            (ARM_LENGTH * ARM_LENGTH);
        // (~2 µs)
        m = (COS_THETA_S[sid] * pivot_x +
             SIN_THETA_S[sid] * pivot_y);
        // (~9 µs)
        n = k / (2 * ARM_LENGTH * sqrt(pivot_z * pivot_z + m * m));

        // Test if other bad things happened.
        // Abort computation of remaining angles if this one is not OK.
        // (~1 µs)
        if (abs(n) >= 1)
        {
            movOK = -2;
            break;
        }

        // Compute servo angle.
        // (22-32 µs This calculation takes half of the total computation time !)
        servo_rad = asin(n) - atan(m / pivot_z);

        // Convert radians to an angle between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
        // (~1 µs)
        servo_rad = this->mapDouble(servo_rad,
                                    -HALF_PI, HALF_PI,
                                    SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Scale values by aggro.
        // (~6 µs)
        servo_rad = SERVO_MID_ANGLE + (servo_rad - SERVO_MID_ANGLE) * AGGRO;

        // Check if the angle is in min/max.
        // Abort computation of remaining angles if this one is not OK.
        // (~1 µs)
        if (servo_rad > SERVO_MAX_ANGLE)
        {
            movOK = -3;
            break;
        }
        else if (servo_rad < SERVO_MIN_ANGLE)
        {
            movOK = -4;
            break;
        }

        // Apply reverse if needed.
        // (~1 µs)
        if (SERVO_REVERSE[sid])
        {
            servo_angles[sid].rad = SERVO_MIN_ANGLE +
                                    SERVO_MAX_ANGLE -
                                    servo_rad;
        }
        else
        {
            servo_angles[sid].rad = servo_rad;
        }

        // Convert radians to degrees.
        // (~2 µs)
        servo_angles[sid].deg = degrees(servo_angles[sid].rad);

        // Convert radians to pulse width.
        // (~5 µs)
        servo_angles[sid].pw = (int)this->mapDouble(servo_angles[sid].rad,
                                                    SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
                                                    SERVO_MIN_US, SERVO_MAX_US);

        // Apply trim values.
        // (~1 µs)
        servo_angles[sid].pw += ANGLE_TRIM[sid];

        // Constrain PW to min/max.
        // (~2 µs)
        servo_angles[sid].pw = (int)constrain(servo_angles[sid].pw,
                                              SERVO_MIN_US, SERVO_MAX_US);
    }

    // Update platform coordinates if there are no errors.
    // (~1 µs)
    if (movOK == 0)
    {
        // Otherwise update platform coordinates.
        _coord.hx_x = coord.hx_x;
        _coord.hx_y = coord.hx_y;
        _coord.hx_z = coord.hx_z;
        _coord.hx_a = coord.hx_a;
        _coord.hx_b = coord.hx_b;
        _coord.hx_c = coord.hx_c;
    }

    return movOK;
}

double Hexapod_Kinematics::getHX_X() { return _coord.hx_x; }
double Hexapod_Kinematics::getHX_Y() { return _coord.hx_y; }
double Hexapod_Kinematics::getHX_Z() { return _coord.hx_z; }
double Hexapod_Kinematics::getHX_A() { return _coord.hx_a; }
double Hexapod_Kinematics::getHX_B() { return _coord.hx_b; }
double Hexapod_Kinematics::getHX_C() { return _coord.hx_c; }
