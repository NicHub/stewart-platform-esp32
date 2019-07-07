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
    Serial.println("\n########## demoMov_MinMaxAllAxis START ##########");

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
    Serial.println("\n########## demoMov_circles START ##########");

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
            // hx_servo.updateServos(movOK, 15UL);
            hx_servo.updateServosIncremental(movOK, 0UL);
        }
    }
    Serial.println("demoMov_circles DONE");
}

/**
 *
 */
void Hexapod_Demo::demoMov_shakeHeave()
{
    Serial.println("\n########## demoMov_shakeHeave START ##########");
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
    Serial.println("\n########## TEST NaN ##########");
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
void Hexapod_Demo::testCalcSpeed()
{
    Serial.println("\n########## TEST CALCULATION SPEED ##########");
    unsigned long T1 = 0, T2 = 0, TTot = 0, dT = 0, Tmin = 10000, Tmax = 0;
    const int nb_intervals = 2;
    const double divide = 3;
    int count = 0;
    for (double sway = HX_X_MIN / divide; sway <= HX_X_MAX / divide; sway += (HX_X_MAX - HX_X_MIN) / nb_intervals / divide)
    {
        for (double surge = HX_Y_MIN / divide; surge <= HX_Y_MAX / divide; surge += (HX_Y_MAX - HX_Y_MIN) / nb_intervals / divide)
        {
            for (double heave = HX_Z_MIN / divide; heave <= HX_Z_MAX / divide; heave += (HX_Z_MAX - HX_Z_MIN) / nb_intervals / divide)
            {
                for (double pitch = HX_A_MIN / divide; pitch <= HX_A_MAX / divide; pitch += (HX_A_MAX - HX_A_MIN) / nb_intervals / divide)
                {
                    for (double roll = HX_B_MIN / divide; roll <= HX_B_MAX / divide; roll += (HX_B_MAX - HX_B_MIN) / nb_intervals / divide)
                    {
                        for (double yaw = HX_C_MIN / divide; yaw <= HX_C_MAX / divide; yaw += (HX_C_MAX - HX_C_MIN) / nb_intervals / divide)
                        {
                            T1 = micros();
                            hx_servo.calcServoAngles({sway, surge, heave, pitch, roll, yaw}, servo_angles);
                            // hx_servo.calcServoAnglesAlgo1({sway, surge, heave, pitch, roll, yaw}, servo_angles);
                            // hx_servo.calcServoAnglesAlgo2({sway, surge, heave, pitch, roll, yaw}, servo_angles);
                            // hx_servo.calcServoAnglesAlgo3({sway, surge, heave, pitch, roll, yaw}, servo_angles);
                            T2 = micros();
                            dT = T2- T1;
                            // Serial.println(dT);
                            if(dT > Tmax) Tmax = dT;
                            else if (dT < Tmin) Tmin = dT;
                            TTot += (dT);
                            count++;
                        }
                    }
                }
            }
        }
    }
    Serial.print("Algorithm                 = ");
    Serial.println(ALGO);
    Serial.print("nb iterations             = ");
    Serial.println(count);
    Serial.print("total time elapsed (us)   = ");
    Serial.println(TTot);
    Serial.print("time per calculation (us) = ");
    Serial.println(TTot / count);
    Serial.print("Tmin (us)                 = ");
    Serial.println(Tmin);
    Serial.print("Tmax (us)                 = ");
    Serial.println(Tmax);
}

/**
 *
 */
void Hexapod_Demo::findMinMax()
{
    double HX_X_MIN = 0;
    double HX_X_MAX = 0;
    // double HX_X_MID = 0;

    double HX_Y_MIN = 0;
    double HX_Y_MAX = 0;
    // double HX_Y_MID = 0;

    double HX_Z_MIN = 0;
    double HX_Z_MAX = 0;
    // double HX_Z_MID = 0;

    double HX_A_MIN = 0;
    double HX_A_MAX = 0;
    // double HX_A_MID = 0;

    double HX_B_MIN = 0;
    double HX_B_MAX = 0;
    // double HX_B_MID = 0;

    double HX_C_MIN = 0;
    double HX_C_MAX = 0;
    // double HX_C_MID = 0;

    unsigned long T1 = millis();
    unsigned long timeToFindMinMax;
    uint8_t movOK;
    angle_t servo_angles[NB_SERVOS];

    double COORD_MIN = -100.0;
    double COORD_MAX = 200.0;
    double COORD_INC = 1;

    for (double coord = COORD_MIN; coord < COORD_MAX; coord += COORD_INC)
    {
        // Find X min/max.
        movOK = hx_servo.calcServoAngles({coord, 0, 0, 0, 0, 0}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_X_MIN)
                HX_X_MIN = coord;
            else if (coord > HX_X_MAX)
                HX_X_MAX = coord;
        }
        // Find Y min/max.
        movOK = hx_servo.calcServoAngles({0, coord, 0, 0, 0, 0}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_Y_MIN)
                HX_Y_MIN = coord;
            else if (coord > HX_Y_MAX)
                HX_Y_MAX = coord;
        }
        // Find Z min/max.
        movOK = hx_servo.calcServoAngles({0, 0, coord, 0, 0, 0}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_Z_MIN)
                HX_Z_MIN = coord;
            else if (coord > HX_Z_MAX)
                HX_Z_MAX = coord;
        }
    }

    COORD_MIN = -HALF_PI;
    COORD_MAX = HALF_PI;
    COORD_INC = radians(1);

    for (double coord = COORD_MIN; coord < COORD_MAX; coord += COORD_INC)
    {
        // Find A min/max.
        movOK = hx_servo.calcServoAngles({0, 0, 0, coord, 0, 0}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_A_MIN)
                HX_A_MIN = coord;
            else if (coord > HX_A_MAX)
                HX_A_MAX = coord;
        }
        // Find B min/max.
        movOK = hx_servo.calcServoAngles({0, 0, 0, 0, coord, 0}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_B_MIN)
                HX_B_MIN = coord;
            else if (coord > HX_B_MAX)
                HX_B_MAX = coord;
        }
        // Find C min/max.
        movOK = hx_servo.calcServoAngles({0, 0, 0, 0, 0, coord}, servo_angles);
        if (movOK == 0)
        {
            if (coord < HX_C_MIN)
                HX_C_MIN = coord;
            else if (coord > HX_C_MAX)
                HX_C_MAX = coord;
        }
    }

    // HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2;
    // HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2;
    // HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2;
    // HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2;
    // HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2;
    // HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2;

    timeToFindMinMax = millis() - T1;

    Serial.println("\n########## MIN / MAX ##########");

    Serial.print("\nHX_X_MIN = ");
    Serial.print(HX_X_MIN);
    Serial.println(" mm");
    Serial.print("HX_X_MAX = ");
    Serial.print(HX_X_MAX);
    Serial.println(" mm");

    Serial.print("\nHX_Y_MIN = ");
    Serial.print(HX_Y_MIN);
    Serial.println(" mm");
    Serial.print("HX_Y_MAX = ");
    Serial.print(HX_Y_MAX);
    Serial.println(" mm");

    Serial.print("\nHX_Z_MIN = ");
    Serial.print(HX_Z_MIN);
    Serial.println(" mm");
    Serial.print("HX_Z_MAX = ");
    Serial.print(HX_Z_MAX);
    Serial.println(" mm");

    Serial.print("\nHX_A_MIN = ");
    Serial.print(degrees(HX_A_MIN));
    Serial.println(" deg");
    Serial.print("HX_A_MAX = ");
    Serial.print(degrees(HX_A_MAX));
    Serial.println(" deg");

    Serial.print("\nHX_B_MIN = ");
    Serial.print(degrees(HX_B_MIN));
    Serial.println(" deg");
    Serial.print("HX_B_MAX = ");
    Serial.print(degrees(HX_B_MAX));
    Serial.println(" deg");

    Serial.print("\nHX_C_MIN = ");
    Serial.print(degrees(HX_C_MIN));
    Serial.println(" deg");
    Serial.print("HX_C_MAX = ");
    Serial.print(degrees(HX_C_MAX));
    Serial.println(" deg");

    Serial.print("\ntimeToFindMinMax = ");
    Serial.print(timeToFindMinMax);
    Serial.println(" ms");
}
