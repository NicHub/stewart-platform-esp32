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

// External libs.
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiiChuck.h>

// Hexapod libs.
#include <Hexapod_Demo.h>
#include <Hexapod_GPIO.h>
#include <Hexapod_Nunchuck.h>
#include <Hexapod_Kinematics.h>
#include <Hexapod_Serial.h>
#include <Hexapod_Servo.h>

// Global variables.
angle_t servo_angles[NB_SERVOS];
Hexapod_Servo hx_servo; // Servo pins are defined in Hexapod_Config_`x`.h (where `x` is the file number)
Hexapod_Serial hx_serial;
Hexapod_Nunchuck hx_nunchuck;
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
#if ENABLE_NUNCHUCK_READ
    hx_nunchuck.setupNunchuck();
#endif
    hx_demo.demoMov_circles(3);

    // Go to home position.
    uint8_t movOK = hx_servo.calcServoAngles({0, 0, 0, 0, 0, 0}, servo_angles);
    hx_servo.updateServos(movOK);

    // Find min and max coord.
    hx_demo.findMinMax();

    // Test calculation speed.
    hx_demo.testCalcSpeed();
}

/**
 *
 */
void loop()
{
#if ENABLE_NUNCHUCK_READ
    hx_nunchuck.stopIfNotConnected();
    hx_nunchuck.nunchuckControl();
#endif

#if ENABLE_SERIAL_READ
    hx_serial.serialRead();
#endif
}
