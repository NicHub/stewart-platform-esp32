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

#ifndef __HEXAPOD_GCODE_H__
#define __HEXAPOD_GCODE_H__

class Hexapod_GCode
{
    // TODO:
    // Currently all the G-Code handling is done in the Hexapod_Serial class.
    // This must be changed so that we have a G-Code class that can be used
    // in communication classes (Serial, Bluetooth, Wifi).
public:
    Hexapod_GCode();
};

#endif
