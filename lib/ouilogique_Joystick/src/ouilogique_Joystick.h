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

#ifndef __OUILOGIQUE_JOYSTICK_h
#define __OUILOGIQUE_JOYSTICK_h

#include <Arduino.h>

class ouilogique_Joystick
{
private:
  const uint8_t pinX;
  const uint8_t pinY;
  const uint8_t pinZ;
  int16_t analog_read_min = 0;
  int16_t analog_read_max = 4095; // 12 bits
  double out_min_x = -0;
  double out_max_x = 0;
  double out_mid_x = (out_max_x + out_min_x) / 2;
  double out_min_y = -0;
  double out_max_y = 0;
  double out_mid_y = (out_max_y + out_min_y) / 2;
  int16_t raw_value_min_x = analog_read_min;
  int16_t raw_value_max_x = analog_read_max;
  int16_t raw_value_mid_x = (raw_value_max_x + raw_value_min_x) / 2;
  int16_t raw_value_min_y = analog_read_min;
  int16_t raw_value_max_y = analog_read_max;
  int16_t raw_value_mid_y = (raw_value_max_y + raw_value_min_y) / 2;
  int16_t dead_band = 400;
  int16_t dead_band_min_x = raw_value_mid_x - dead_band;
  int16_t dead_band_max_x = raw_value_mid_x + dead_band;
  int16_t dead_band_min_y = raw_value_mid_y - dead_band;
  int16_t dead_band_max_y = raw_value_mid_y + dead_band;

public:
  /*
     * ======== MAIN FUNCTIONS ==========
     */
  ouilogique_Joystick(uint8_t pinX, uint8_t pinY, uint8_t pinZ);
  int16_t getRawX();
  int16_t getRawY();
  double getX();
  double getY();
  bool getZ();
  void calibrate();
  int16_t getRawValueMidX();
  int16_t getRawValueMidY();
  void setLimits(
      double out_min_x,
      double out_max_x,
      double out_mid_x,
      double out_min_y,
      double out_max_y,
      double out_mid_y);

  /*
     * ======== HELPER FUNCTION ==========
     */
  double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
};

#endif
