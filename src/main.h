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

#ifndef __MAIN_H__
#define __MAIN_H__

// Usefull functions.
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))
#define SET_LED digitalWrite(LED_BUILTIN, LOW);
#define CLEAR_LED digitalWrite(LED_BUILTIN, HIGH);

// External libs.
#include <Arduino.h>
#include <P19.h>
#include <ESP32Servo.h>

// Joystick lib.
#define X_PIN 26
#define Y_PIN 12
#define Z_PIN 32
#include <ouilogique_Joystick.h>
#include <HexapodJoystick.h>

// Hexapod libs.
#include <HexapodKinematics.h>
#include <HexapodServo.h>
#include <HexapodSerial.h>
#include <HexapodDemo.h>

#endif //__MAIN_H__
