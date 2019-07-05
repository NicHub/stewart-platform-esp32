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
 * Calculation of the servo angles in radians, degrees and in microseconds (PWM)
 * given the desired target platform coordinates.
 *
 * @param coord : the desired target platform coordinates.
 * A struct containing X (sway), Y (surge), Z (heave) in mm
 * and A (pitch), B (roll) and C (yaw) in radians.
 *
 * @param servo_angles : pointer to an array of struct containing
 * the calculated servos angles in radians, degrees and in microseconds (PWM).
 *
 * @return
 * Returns = 0 if OK
 * Returns < 0 if Error
 *
 */
int8_t Hexapod_Kinematics::calcServoAngles(platform_t coord, angle_t *servo_angles)
{
    int8_t movOK = 0;

    // Algorithm 1 takes ~275 µs / movement.
    // Algorithm 2 takes ~707 µs / movement.
#if ALGO == 1
    movOK = calcServoAnglesAlgo1(coord, servo_angles);
#elif ALGO == 2
    movOK = calcServoAnglesAlgo2(coord, servo_angles);
#endif
    return movOK;
}

/**
 *
 */
int8_t Hexapod_Kinematics::calcServoAnglesAlgo1(platform_t coord, angle_t *servo_angles)
{
    double BP_x, BP_y, BP_z, // Platform joint coordinates relative to servo pivot.
        BP2,                 // Distance^2 between servo pivot (B) and platform joint (P).
        s, t;                // Intermediate values.

    angle_t new_servo_angles[NB_SERVOS];

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

    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        // Compute the new platform joint coordinates relative to servo pivot.
        // (~7 µs)
        BP_x = P_COORDS[sid][0] * cosB * cosC +
               P_COORDS[sid][1] * (sinA * sinB * cosC - cosA * sinC) +
               coord.hx_x -
               B_COORDS[sid][0];
        BP_y = P_COORDS[sid][0] * cosB * sinC +
               P_COORDS[sid][1] * (sinA * sinB * sinC + cosA * cosC) +
               coord.hx_y -
               B_COORDS[sid][1];
        BP_z = -P_COORDS[sid][0] * sinB +
               P_COORDS[sid][1] * sinA * cosB +
               coord.hx_z -
               Z_HOME;

        // Distance^2 between servo pivot (B) and platform joint (P).
        BP2 = POW(BP_x, 2) + POW(BP_y, 2) + POW(BP_z, 2);

        // Test if the new distance between servo pivot and platform joint
        // is longer than physically possible.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (BP2 > BP2_MAX)
        {
            movOK = -1;
            break;
        }

        // Calculation of intermediate values.
        // (~2 µs)
        t = (COS_THETA_S[sid] * BP_x +
             SIN_THETA_S[sid] * BP_y) /
            BP_z;

        // If t <= -1, then s is undefined. This case is almost impossible
        // in practice, so maybe the test should be removed.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (t <= -1)
        {
            movOK = -5;
            break;
        }

        // Mathematically speaking, we should also test if BP_z == 0 before calculating s.
        // But BP_z == 0 is impossible in practice.
        // (~9 µs)
        s = (BP2 - BP2_PERP) /
            (2 * ARM_LENGTH * BP_z * sqrt(1 + t * t));

        // Tests if there are no physically possible solutions.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (abs(s) > 1)
        {
            movOK = -2;
            break;
        }

        // Compute servo angle.
        // (~40 µs This calculation takes 2/3 of the total computation time !)
        new_servo_angles[sid].rad = asin(s) - atan(t);

        // Rotate the angle.
        // (~1 µs)
        new_servo_angles[sid].rad += SERVO_HALF_ANGULAR_RANGE;

        // Convert radians to degrees.
        // (~2 µs)
        new_servo_angles[sid].deg = degrees(new_servo_angles[sid].rad);

        // Convert radians to microseconds (PWM).
        // The calibration values take into account the fact
        // that the odd and even arms are a reflection of each other.
        // (~5 µs)
        new_servo_angles[sid].us =
            SERVO_CALIBRATION[sid].gain * new_servo_angles[sid].rad +
            SERVO_CALIBRATION[sid].offset;

        // Check if the angle is in min/max.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (new_servo_angles[sid].us > SERVO_MAX_US)
        {
            movOK = -3;
            break;
        }
        else if (new_servo_angles[sid].us < SERVO_MIN_US)
        {
            movOK = -4;
            break;
        }

#if false
        // Calculate Z_HOME so that the arm is horizontal.
        // The result is valid only for the home position
        // (first line of table calculated by desktop app).
        double Z_HOME_CALC =
            BP_z - sqrt(POW(ROD_LENGTH, 2) -
                        POW((ARM_LENGTH - (COS_THETA_S[sid] * BP_x + SIN_THETA_S[sid] * BP_y)), 2) -
                        POW((-SIN_THETA_S[sid] * BP_x + COS_THETA_S[sid] * BP_y), 2));
        Z_HOME_CALC += Z_HOME;
        new_servo_angles[sid].debug = Z_HOME_CALC;
#endif
    }

    // Update platform coordinates if there are no errors.
    // (~1 µs)
    if (movOK == 0)
    {
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servo_angles[sid] = new_servo_angles[sid];
        }
        _coord.hx_x = coord.hx_x;
        _coord.hx_y = coord.hx_y;
        _coord.hx_z = coord.hx_z;
        _coord.hx_a = coord.hx_a;
        _coord.hx_b = coord.hx_b;
        _coord.hx_c = coord.hx_c;
    }

    return movOK;
}

/**
 *
 */
int8_t Hexapod_Kinematics::calcServoAnglesAlgo2(platform_t coord, angle_t *servo_angles)
{
    double BP_x, BP_y, BP_z,            // Platform joint coordinates relative to servo pivot.
        BP2,                            // Distance^2 between platform joint and servo pivot.
        i0, i1, i2, i3, i4, i5, i6, i7; // Intermediate values.

    angle_t new_servo_angles[NB_SERVOS];

    // Intermediate values, to avoid recalculating sin and cos.
    // (3 µs).
    double cosA = cos(coord.hx_a),
           cosB = cos(coord.hx_b),
           cosC = cos(coord.hx_c),
           sinA = sin(coord.hx_a),
           sinB = sin(coord.hx_b),
           sinC = sin(coord.hx_c),
           ARM_LENGTH_2 = POW(ARM_LENGTH, 2),
           ROD_LENGTH_2 = POW(ROD_LENGTH, 2);

    // Assume everything will be OK.
    int8_t movOK = 0;

    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        // Intermediate values, to avoid recalculating sin and cos.
        double cosCpD = cos(coord.hx_c + M_THETA_S[sid]),
               sinCpD = sin(coord.hx_c + M_THETA_S[sid]);

        BP_x = coord.hx_x * cosD[sid] -
               coord.hx_y * sinD[sid] -
               B_COORDS[sid][0] * cosD[sid] +
               B_COORDS[sid][1] * sinD[sid] +
               P_COORDS[sid][0] * cosB * cosCpD +
               P_COORDS[sid][1] * (cosC * (sinA * sinB * cosD[sid] - cosA * sinD[sid]) -
                                   sinC * (cosA * cosD[sid] + sinA * sinB * sinD[sid]));
        BP_y = coord.hx_x * sinD[sid] +
               coord.hx_y * cosD[sid] -
               B_COORDS[sid][0] * sinD[sid] -
               B_COORDS[sid][1] * cosD[sid] +
               P_COORDS[sid][0] * cosB * sinCpD +
               P_COORDS[sid][1] * (cosA * cosCpD + sinA * sinB * sinCpD);
        BP_z = coord.hx_z -
               Z_HOME -
               P_COORDS[sid][0] * sinB +
               P_COORDS[sid][1] * sinA * cosB;

        double BP_x2 = POW(BP_x, 2),
               BP_y2 = POW(BP_y, 2),
               BP_z2 = POW(BP_z, 2);

        // Square of the new distance between platform joint and servo pivot.
        BP2 = BP_x2 + BP_y2 + BP_z2;

        // Test if the new distance between servo pivot and platform joint
        // is longer than physically possible.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (BP2 > BP2_MAX)
        {
            movOK = -1;
            break;
        }

        // (~5 µs)
        i0 = BP_x2 + BP_z2;
        i1 = i0 + BP_y2 - ROD_LENGTH_2;
        i2 = i0 - BP_y2 + ROD_LENGTH_2;

        // (~49 µs)
        i3 = -(BP_z2 * (POW(ARM_LENGTH_2, 2) + POW(i1, 2) - 2 * ARM_LENGTH_2 * i2));

        // (~51 µs)
        i4 = sqrt(i3);

        // (~117 µs)
        i5 = (ARM_LENGTH_2 * BP_z2 + BP_x2 * BP_z2 +
              BP_y2 * BP_z2 + POW(BP_z, 4) - BP_z2 * ROD_LENGTH_2 -
              BP_x * i4);

        // (~32 µs)
        i6 = (BP_z * (ARM_LENGTH_2 * BP_x + POW(BP_x, 3) + BP_x * BP_y2 +
                      BP_x * BP_z2 - BP_x * ROD_LENGTH_2 + i4));

        i7 = i5 / i6;

        // Compute servo angle.
        // (~22 µs)
        new_servo_angles[sid].rad = atan(i7);

        // Rotate the angle.
        // (~1 µs)
        new_servo_angles[sid].rad += SERVO_HALF_ANGULAR_RANGE;

        // Convert radians to degrees.
        // (~2 µs)
        new_servo_angles[sid].deg = degrees(new_servo_angles[sid].rad);

        // Convert radians to microseconds (PWM).
        // The calibration values take into account the fact
        // that the odd and even arms are a reflection of each other.
        // (~5 µs)
        new_servo_angles[sid].us =
            SERVO_CALIBRATION[sid].gain * new_servo_angles[sid].rad +
            SERVO_CALIBRATION[sid].offset;

        // Check if the angle is in min/max.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (new_servo_angles[sid].us > SERVO_MAX_US)
        {
            movOK = -3;
            break;
        }
        else if (new_servo_angles[sid].us < SERVO_MIN_US)
        {
            movOK = -4;
            break;
        }
    }

    // Update platform coordinates if there are no errors.
    // (~1 µs)
    if (movOK == 0)
    {
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servo_angles[sid] = new_servo_angles[sid];
        }
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
