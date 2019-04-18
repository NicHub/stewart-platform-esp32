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

#include <Hexapod_Demo.h>

extern Hexapod_Kinematics hk;            // Stewart platform object.
extern servo_t servo_angles[NB_SERVOS]; // Servo angles.
extern Servo servos[NB_SERVOS];         // Servo objects.

/**
 *
 */
void demoMov_MinMaxAllAxis()
{
    Serial.println("demoMov_MinMaxAllAxis START");
    const platform_t coords[] = {
        // sway
        {SWAY_MAX, 0, 0, 0, 0, 0},
        {SWAY_MIN, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // surge
        {0, SURGE_MAX, 0, 0, 0, 0},
        {0, SURGE_MIN, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // heave
        {0, 0, HEAVE_MAX, 0, 0, 0},
        {0, 0, HEAVE_MIN, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // pitch
        {0, 0, 0, PITCH_MAX, 0, 0},
        {0, 0, 0, PITCH_MIN, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // roll
        {0, 0, 0, 0, ROLL_MAX, 0},
        {0, 0, 0, 0, ROLL_MIN, 0},
        {0, 0, 0, 0, 0, 0},

        // yaw
        {0, 0, 0, 0, 0, YAW_MAX},
        {0, 0, 0, 0, 0, YAW_MIN},
        {0, 0, 0, 0, 0, 0}};

    int8_t movOK = -1;
    for (uint8_t cnt = 0; cnt < COUNT_OF(coords); cnt++)
    {
        Serial.print("cnt = ");
        Serial.println(cnt);
        movOK = hk.calcServoAngles(servo_angles, coords[cnt]);
        updateServos(movOK);
        delay(1000);
    }
    Serial.println("demoMov_MinMaxAllAxis DONE");
}

/**
 *
 */
void demoMov_circles(uint8_t nb_turn = 1)
{
    // Move in circles in the horizontal plane.

    Serial.println("demoMov_circles START");

    const uint8_t nb_points = 90;
    const double radius = SWAY_MAX;
    const double angleInc = TWO_PI / nb_points;
    double angle = 0;
    platform_t coords[nb_points];
    for (uint8_t angleID = 0; angleID < nb_points; angleID++)
    {
        coords[angleID] = {(radius * sin(angle)),
                           (radius * cos(angle)),
                           0, 0, 0, 0};
        angle += angleInc;
    }
    int8_t movOK = -1;
    for (uint8_t turn = 0; turn < nb_turn; turn++)
    {
        for (uint8_t cnt = 0; cnt < nb_points; cnt++)
        {
            movOK = hk.calcServoAngles(servo_angles, coords[cnt]);
            updateServos(movOK);
            delay(8);
        }
    }
    Serial.println("demoMov_circles DONE");
}

/**
 *
 */
void demoMov_shake()
{
    Serial.println("demoMov_shake START");
    double demoMov_shakeZ = HEAVE_MIN;
    int8_t movOK = -1;
    const uint32_t wait = 200;

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    for (uint8_t demoMov_shake = 0; demoMov_shake < 10; demoMov_shake++)
    {
        delay(60);
        movOK = hk.calcServoAngles(servo_angles, {0, 0, demoMov_shakeZ, 0, 0, 0});
        updateServos(movOK);
        demoMov_shakeZ = -demoMov_shakeZ;
    }

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.home(servo_angles);
    updateServos(movOK);

    Serial.println("demoMov_shake DONE");
}

/**
 *
 */
void testNaN()
{
    Serial.print("\nTEST NaN");
    int8_t movOK;
    platform_t coords[] = {
        {-33.0, 11.0, -15.0, 17.0, -5.7, 5.7},
        {-33.0, 11.0, -15.0, 5.7, 17.0, 17.0}};
    for (uint8_t coord_id = 0; coord_id < COUNT_OF(coords); coord_id++)
    {
        movOK = hk.calcServoAngles(servo_angles, coords[coord_id]);
        printServoAngles();
        Serial.print("movOK = ");
        Serial.println(movOK);
    }
}

/**
 *
 */
void testCalculations()
{
    hk.calcServoAngles(servo_angles, {0, 0, 0, 0, 0, 0});
    Serial.print("\n0, 0, 0, 0, 0, 0 ");
    printServoAngles();

    hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    Serial.print("\n0, 0, HEAVE_MAX, 0, 0, 0");
    printServoAngles();

    hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    Serial.print("\n0, 0, HEAVE_MIN, 0, 0, 0");
    printServoAngles();
}
