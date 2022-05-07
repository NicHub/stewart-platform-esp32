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
#include <Hexapod_Servo.h>

#define MAX_BUF 64 // Serial buffer size

class Hexapod_Serial
{
private:
    char buffer[MAX_BUF];          // where we store the message until we get a ';'
    int sofar = 0;                 // how much is in the buffer
    double px, py, pz, pa, pb, pc; // positions
    char mode_abs = 1;             // absolute mode?
    long line_number = 0;

    void ready();
    void processCommand();
    double parseNumber(const char code, double val);
    void output(const char code, double val);
    void G0(double newx, double newy, double newz, double newa, double newb, double newc);
    void G4P(double seconds);
    void G90();
    void G91();
    void M100();
    void M114();

public:
    Hexapod_Serial();
    void setupSerial();
    void serialRead();
};
