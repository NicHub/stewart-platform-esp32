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
#include <WiiChuck.h>
#include <Hexapod_Servo.h>

// nunchuck_t
typedef struct
{
    double joy_x;
    double joy_y;
    bool btn_c;
    bool btn_z;
    double acc_x;
    double acc_y;
    double acc_z;
    double roll_angle;
    double pitch_angle;
} nunchuck_t;

class Hexapod_Nunchuck : public Accessory
{
private:
    double nckXmin = HX_X_MIN;
    double nckXmax = HX_X_MAX;
    double nckYmin = HX_Y_MIN;
    double nckYmax = HX_Y_MAX;
    double nckAmin = HX_A_MIN;
    double nckAmax = HX_A_MAX;
    double nckBmin = HX_B_MIN;
    double nckBmax = HX_B_MAX;

public:
    Hexapod_Nunchuck();
    int setupNunchuck();
    void nunchuckControl();
    bool connected();
    int readNunchuck(nunchuck_t *nckVal);
};

class Average
{
private:
    static const uint8_t nbPoints = 10;
    uint8_t index = 0;
    double sum = 0;
    double valHistory[nbPoints];

public:
    Average();
    double MovingAverage(double val, double in_low, double in_high, double out_low, double out_high);
};
