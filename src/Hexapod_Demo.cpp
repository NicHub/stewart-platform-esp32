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

#include <Hexapod_Demo.h>
#include <Hexapod_Servo.h>
#include <Hexapod_Kinematics.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

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
        // X sway
        {HX_X_MAX, 0, 0, 0, 0, 0},
        {HX_X_MIN, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Y surge
        {0, HX_Y_MAX, 0, 0, 0, 0},
        {0, HX_Y_MIN, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Z heave
        {0, 0, HX_Z_MAX, 0, 0, 0},
        {0, 0, HX_Z_MIN, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // A pitch
        {0, 0, 0, HX_A_MAX, 0, 0},
        {0, 0, 0, HX_A_MIN, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // B roll
        {0, 0, 0, 0, HX_B_MAX, 0},
        {0, 0, 0, 0, HX_B_MIN, 0},
        {0, 0, 0, 0, 0, 0},

        // C yaw
        {0, 0, 0, 0, 0, HX_C_MAX},
        {0, 0, 0, 0, 0, HX_C_MIN},
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
    const double radius = HX_X_MAX;
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
            static unsigned long T1;
            while ((millis() - T1) < 8UL)
            {
            }
            T1 = millis();
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles);
            hx_servo.updateServos(movOK, 0UL);
        }
    }
    Serial.println("demoMov_circles DONE");
}

/**
 *
 */
void Hexapod_Demo::demoMov_shakeHeave()
{
    Serial.println("demoMov_shakeHeave START");
    double shakeVal = HX_Z_MIN;
    int8_t movOK = -1;
    const uint32_t wait = 200;

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HX_Z_MAX, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HX_Z_MIN, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    for (uint8_t demoMov_shakeHeave = 0; demoMov_shakeHeave < 10; demoMov_shakeHeave++)
    {
        static unsigned long T1;
        while ((millis() - T1) < 1500UL) // 60UL for fast servos.
        {
        }
        T1 = millis();
        movOK = hx_servo.calcServoAngles({0, 0, shakeVal, 0, 0, 0}, servo_angles);
        hx_servo.updateServos(movOK);
        shakeVal = -shakeVal;
    }

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HX_Z_MAX, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.calcServoAngles({0, 0, HX_Z_MIN, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    delay(wait);
    movOK = hx_servo.home(servo_angles);
    hx_servo.updateServos(movOK);

    Serial.println("demoMov_shakeHeave DONE");
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

    hx_servo.calcServoAngles({0, 0, HX_Z_MAX, 0, 0, 0}, servo_angles);
    Serial.print("\n0, 0, HX_Z_MAX, 0, 0, 0");
    hx_servo.printServoAngles();

    hx_servo.calcServoAngles({0, 0, HX_Z_MIN, 0, 0, 0}, servo_angles);
    Serial.print("\n0, 0, HX_Z_MIN, 0, 0, 0");
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
        hx_servo.calcServoAngles({0, 0, HX_Z_MAX, 0, 0, 0}, servo_angles);
        T2 = micros();
        TTot += (T2 - T1);
    }
    Serial.print("total time elapsed (us) = ");
    Serial.println(TTot);
    Serial.print("time per calculation (us) = ");
    Serial.println(TTot / nb_iter);
}
