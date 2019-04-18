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

extern angle_t servo_angles[NB_SERVOS];
extern Hexapod_Servo hx_servo;

/**
 *
 */
Hexapod_Demo::Hexapod_Demo()
{
}

/**
 *
 */
void Hexapod_Demo::demoMov_MinMaxAllAxis()
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
        movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles);
        hx_servo.updateServos(movOK);
        delay(1000);
    }
    Serial.println("demoMov_MinMaxAllAxis DONE");
}

/**
 *
 */
void Hexapod_Demo::demoMov_circles(uint8_t nb_turn = 1)
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
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles);
            hx_servo.updateServos(movOK);
            delay(8);
        }
    }
    Serial.println("demoMov_circles DONE");
}

/**
 *
 */
void Hexapod_Demo::demoMov_shake()
{
    Serial.println("demoMov_shake START");
    double demoMov_shakeZ = HEAVE_MIN;
    int8_t movOK = -1;
    const uint32_t wait = 200;

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HEAVE_MAX, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HEAVE_MIN, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    for (uint8_t demoMov_shake = 0; demoMov_shake < 10; demoMov_shake++)
    {
        delay(60);
        movOK = hx_servo.calcServoAngles({0, 0, demoMov_shakeZ, 0, 0, 0}, servo_angles);
        hx_servo.updateServos(movOK);
        demoMov_shakeZ = -demoMov_shakeZ;
    }

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HEAVE_MAX, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HEAVE_MIN, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.home(servo_angles);
    hx_servo.updateServos(movOK);

    Serial.println("demoMov_shake DONE");
}

/**
 *
 */
void Hexapod_Demo::testNaN()
{
    Serial.print("\nTEST NaN");
    int8_t movOK;
    platform_t coords[] = {
        {-33.0, 11.0, -15.0, 17.0, -5.7, 5.7},
        {-33.0, 11.0, -15.0, 5.7, 17.0, 17.0}};
    for (uint8_t coord_id = 0; coord_id < COUNT_OF(coords); coord_id++)
    {
        movOK = hx_servo.calcServoAngles(coords[coord_id], servo_angles);
        hx_servo.printServoAngles();
        Serial.print("movOK = ");
        Serial.println(movOK);
    }
}

/**
 *
 */
void Hexapod_Demo::testCalculations()
{
    hx_servo.calcServoAngles({0, 0, 0, 0, 0, 0}, servo_angles);
    Serial.print("\n0, 0, 0, 0, 0, 0 ");
    hx_servo.printServoAngles();

    hx_servo.calcServoAngles({0, 0, HEAVE_MAX, 0, 0, 0}, servo_angles);
    Serial.print("\n0, 0, HEAVE_MAX, 0, 0, 0");
    hx_servo.printServoAngles();

    hx_servo.calcServoAngles({0, 0, HEAVE_MIN, 0, 0, 0}, servo_angles);
    Serial.print("\n0, 0, HEAVE_MIN, 0, 0, 0");
    hx_servo.printServoAngles();
}


/**
 *
 */
void Hexapod_Demo::testCalcSpeed(uint16_t nb_iter)
{
    Serial.println("TEST CALCULATION SPEED");
    Serial.print("Number of iterations = ");
    Serial.println(nb_iter);
    unsigned long T1 = 0, T2 = 0, TTot = 0;
    for (uint16_t cid = 0; cid < nb_iter; cid++)
    {
        T1 = micros();
        hx_servo.calcServoAngles({0, 0, HEAVE_MAX, 0, 0, 0}, servo_angles);
        T2 = micros();
        TTot += (T2 - T1);
    }
    Serial.print("total time elapsed (us) = ");
    Serial.println(TTot);
    Serial.print("time per calculation (us) = ");
    Serial.println(TTot / nb_iter);
}
