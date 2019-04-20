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

// Servo pins are defined in
// Hexapod_Config_`x`.h
// (where `x` is the file number)

// Joystick pins.
#define X_PIN 26
#define Y_PIN 12
#define Z_PIN 32

// Global variables.
angle_t servo_angles[NB_SERVOS];
Hexapod_Servo hx_servo;
Hexapod_Serial hx_serial;
Hexapod_Joystick hx_joystick(X_PIN, Y_PIN, Z_PIN);
Hexapod_Demo hx_demo;
Hexapod_GPIO hx_gpio;

/**
 *
 */
void setup()
{
    // Setup.
    hx_gpio.setupGPIO();
    hx_serial.setupSerial();
    hx_servo.setupServo();
    hx_joystick.setupJoystick();

    // Demo movements.
    hx_demo.demoMov_circles(3);
    hx_demo.demoMov_shakeHeave();
    // hx_demo.demoMov_MinMaxAllAxis();
    hx_demo.testCalcSpeed(2);
}

/**
 *
 */
void loop()
{
#if ENABLE_JOYSTICK_READ
    hx_joystick.joystickControl();
#endif

#if ENABLE_SERIAL_READ
    hx_serial.serialControl();
#endif
}
