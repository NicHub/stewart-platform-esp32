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

// External libs.
#include <Arduino.h>
#include <ESP32Servo.h>

// Hexapod libs.
#include <Hexapod_Joystick.h>
#include <Hexapod_Kinematics.h>
#include <Hexapod_Servo.h>
#include <Hexapod_Serial.h>
#include <Hexapod_GPIO.h>
#include <Hexapod_Demo.h>

#endif //__MAIN_H__
