/**
 * O U I L O G I Q U E    J O Y S T I C K
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

#include "ouilogique_Joystick.h"

/**
 *
 */
double ouilogique_Joystick::mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 *
 */
ouilogique_Joystick::ouilogique_Joystick(
    uint8_t pinX, uint8_t pinY, uint8_t pinZ) : pinX(pinX),
                                                pinY(pinY),
                                                pinZ(pinZ)
{
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinZ, INPUT_PULLUP);
}

/**
 *
 */
void ouilogique_Joystick::calibrate()
{
    // nbPoints max = (2^16 / 2) / (2^12) = 8
    const size_t nbPoints = 5;
    const long wait = 10;

    int16_t sumX = 0;
    int16_t sumY = 0;
    for (size_t cnt = 0; cnt < nbPoints; cnt++)
    {
        sumX += getRawX();
        sumY += getRawY();
        delay(wait);
    }
    raw_value_mid_x = sumX / nbPoints;
    raw_value_mid_y = sumY / nbPoints;
}

/**
 *
 */
int16_t ouilogique_Joystick::getRawValueMidX()
{
    return raw_value_mid_x;
}

/**
 *
 */
int16_t ouilogique_Joystick::getRawValueMidY()
{
    return raw_value_mid_y;
}

/**
 *
 */
int16_t ouilogique_Joystick::getRawX()
{
    int16_t rawValue = analogRead(pinX);
    return rawValue;
}

/**
 *
 */
int16_t ouilogique_Joystick::getRawY()
{
    int16_t rawValue = analogRead(pinY);
    return rawValue;
}

/**
 *
 */
double ouilogique_Joystick::getX()
{
    double rawValue = analogRead(pinX);
    double value = 0;
    if (rawValue >= dead_band_min_x && rawValue <= dead_band_max_x)
        value = out_mid_x;
    else if (rawValue >= raw_value_mid_x)
        value = mapDouble(rawValue, raw_value_max_x, raw_value_mid_x, out_min_x, out_mid_x);
    else if (rawValue < raw_value_mid_x)
        value = mapDouble(rawValue, raw_value_mid_x, raw_value_min_x, out_mid_x, out_max_x);
    return value;
}

/**
 *
 */
double ouilogique_Joystick::getY()
{
    double rawValue = analogRead(pinY);
    double value = 0;
    if (rawValue >= dead_band_min_y && rawValue <= dead_band_max_y)
        value = out_mid_y;
    else if (rawValue >= raw_value_mid_y)
        value = mapDouble(rawValue, raw_value_max_y, raw_value_mid_y, out_min_y, out_mid_y);
    else if (rawValue < raw_value_mid_y)
        value = mapDouble(rawValue, raw_value_mid_y, raw_value_min_y, out_mid_y, out_max_y);
    return value;
}

/**
 *
 */
bool ouilogique_Joystick::getZ()
{
    bool value = !digitalRead(pinZ);
    return value;
}

/**
 *
 */
void ouilogique_Joystick::setLimits(
    double out_min_x,
    double out_max_x,
    double out_mid_x,
    double out_min_y,
    double out_max_y,
    double out_mid_y)
{
    this->out_min_x = out_min_x;
    this->out_mid_x = out_mid_x;
    this->out_max_x = out_max_x;
    this->out_min_y = out_min_y;
    this->out_mid_y = out_mid_y;
    this->out_max_y = out_max_y;
}
