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

#pragma once

#include <Arduino.h>
#include <PCA9685.h>
#include <Hexapod_Kinematics.h>

class Hexapod_Servo : public Hexapod_Kinematics
{
private:
    // Servo servos[NB_SERVOS]; // Array of servo objects.
    PCA9685 pwmController;

public:
    Hexapod_Servo();
    void setupServo();
    void updateServosIncremental(int8_t movOK, unsigned long safetyWait_ms = 0UL);
    void updateServos(int8_t movOK, unsigned long safetyWait_ms = 0UL);
    void printServoAngles();
    void printJointAndServoAxisCoord();
};
