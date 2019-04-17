/**
 * S T E W A R T    P L A T F O R M    O N    E S P 1 2
 *
 * Based on
 * 6dof-stewduino
 * Copyright (C) 2018  Philippe Desrosiers
 * https://github.com/xoxota99/stewy
 *
 * Derived from the work of Daniel Waters
 * https://www.youtube.com/watch?v=1jrP1_1ML9M
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
 * the Free Software Foundation, either version 1 of the License, or
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

#include <main.h>

// Global variables.
HexapodKinematics hk;            // Stewart platform object.
servo_t servo_angles[NB_SERVOS]; // Servo angles.
Servo servos[NB_SERVOS];         // Servo objects.

/**
 *
 */
void setupGPIO()
{
    pinMode(LED_BUILTIN, OUTPUT);
    CLEAR_LED;
}

/**
 *
 */
void setup()
{
    setupSerial();
    setupServos();
    setupJoystick();
    setupGPIO();
    demoMov_circles(3);
    demoMov_shake();
    // demoMov_MinMaxAllAxis();
    // testNaN();
    // testCalculations();
    // platform_t coords = {0, 0, HEAVE_MAX, 0, 0, 0};
    // int8_t movOK = hk.calcServoAngles(servo_angles, coords);
    // printServoAngles();
    // updateServos(movOK);
}

/**
 *
 */
void loop()
{
#if ENABLE_JOYSTICK_READ
    joystickControl();
#endif

#if ENABLE_SERIAL_READ
    serialControl();
#endif
}
