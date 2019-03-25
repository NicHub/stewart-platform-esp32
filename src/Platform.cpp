/*
   6dof-stewduino
   Copyright (C) 2018  Philippe Desrosiers

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   Derived from the work of Daniel Waters, https://www.youtube.com/watch?v=1jrP3_1ML9M
*/

#include "Platform.h"
#include "Logger.h"

bool Platform::home(float *servoValues) {
  return moveTo (servoValues, 0, 0, 0, 0, 0, 0); //HOME position. No rotation, no translation.
}

/*
 * Move to a given sway, surge, heave, pitch, roll and yaw values.
 * We expect pitch, roll and yaw in degrees.
 */
bool Platform::moveTo(float *servoValues, int sway, int surge, int heave, float pitch, float roll, float yaw) {

  /*
     TODO:
     1) Understand the "geometry shenanigans" below (k, l, m values).
     2) optimize so we don't run through this loop if we know we're already at the desired setpoint(s).
  */

  double pivot_x, pivot_y, pivot_z,   //Global XYZ coordinates of platform pivot points
         d2,                  //Distance^2 between servo pivot and platform link.
         k, l, m,             //intermediate values
         servo_rad,           //Angle (radians) to turn each servo.
         servo_deg;           //Angle (in degrees) to turn each servo.

  float oldValues[6];

  // intermediate values, to avoid recalculating SIN / COS

  double cr = cos(radians(roll)),
         cp = cos(radians(pitch)),
         cy = cos(radians(yaw)),
         sr = sin(radians(roll)),
         sp = sin(radians(pitch)),
         sy = sin(radians(yaw));

  for (int i = 0; i < 6; i++) {
    oldValues[i] = servoValues[i];
  }

  bool bOk = true;

  for (int i = 0; i < 6; i++) {
    pivot_x = P_COORDS[i][0] * cr * cy + P_COORDS[i][1] * (sp * sr * cr - cp * sy) + sway;
    pivot_y = P_COORDS[i][0] * cr * sy + P_COORDS[i][1] * (cp * cy + sp * sr * sy) + surge;
    pivot_z = -P_COORDS[i][0] * sr + P_COORDS[i][1] * sp * cr + Z_HOME + heave;

    d2 = pow(pivot_x - B_COORDS[i][0], 2) + pow(pivot_y - B_COORDS[i][1], 2) + pow(pivot_z, 2);

    //Geometry shenanigans
    k = d2 - (pow(ROD_LENGTH, 2) - pow(ARM_LENGTH, 2));
    l = 2 * ARM_LENGTH * pivot_z;
    m = 2 * ARM_LENGTH * (cos(THETA_S[i]) * (pivot_x - B_COORDS[i][0]) + sin(THETA_S[i]) * (pivot_y - B_COORDS[i][1]));
    servo_rad = asin(k / sqrt(l * l + m * m)) - atan(m / l);
    //convert radians to an angle between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE
    servo_deg = map(degrees(servo_rad),-90,90,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE);

    if (sqrt(d2) > (ARM_LENGTH + ROD_LENGTH) //the required virtual arm length is longer than physically possible
        || abs(k / (sqrt(l * l + m * m))) >= 1) { //some other bad stuff happened.
      //bad juju.
      Logger::error("Asymptotic condition at i=%d",i);
      Logger::info("abs(k/(sqrt(l*l+m*m))) = %.2f",abs(k / (sqrt(l * l + m * m))));
      Logger::info("sqrt(d2)>(ARM_LENGTH+ROD_LENGTH) = %s",(sqrt(d2) > (ARM_LENGTH + ROD_LENGTH)) ? "true" : "false");

#ifdef SLAM
      servo_deg = SERVO_MAX_ANGLE;  //BUG: Not correct. servo_deg should be one of SERVO_MAX_ANGLE or SERVO_MIN_ANGLE. need to figure out which one, rather than assuming SERVO_MAX_ANGLE.
#else
      // bOk = false;
      //do nothing with this servo. We assume that it's current position is "close enough" (Not sure this is safe, but so far it works).
      servo_deg=servoValues[i];
#endif
      break;
    } else if (servo_deg > SERVO_MAX_ANGLE) {
      servo_deg = SERVO_MAX_ANGLE;
    } else if (servo_deg < SERVO_MIN_ANGLE) {
      servo_deg = SERVO_MIN_ANGLE;
    }

    servoValues[i] = servo_deg;
  }

  if (bOk) {
    _sp_sway = sway;
    _sp_surge = surge;
    _sp_heave = heave;
    _sp_pitch = pitch;
    _sp_roll = roll;
    _sp_yaw = yaw;

    //scale values by aggro.
    for (int i = 0; i < 6; i++) {
      int diff = servoValues[i]-SERVO_MID_ANGLE;

      servoValues[i] = SERVO_MID_ANGLE + (diff * AGGRO);
      servoValues[i] = constrain(servoValues[i],SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      }
  } else {
    //reset back to old values.
    for (int i = 0; i < 6; i++) {
      servoValues[i] = oldValues[i];
    }
  }

  return bOk;
}

/*
 * Move to a given pitch / roll angle (in degrees)
 */
bool Platform::moveTo(float *servoValues, float pitch, float roll) {
  return moveTo(servoValues, _sp_sway, _sp_surge, _sp_heave, pitch, roll, _sp_yaw);
}

int Platform::getSway() {
  return _sp_sway;
}
int Platform::getSurge() {
  return _sp_surge;
}
int Platform::getHeave() {
  return _sp_heave;
}

float Platform::getPitch() {
  return _sp_pitch;
}
float Platform::getRoll() {
  return _sp_roll;
}
float Platform::getYaw() {
  return _sp_yaw;
}
