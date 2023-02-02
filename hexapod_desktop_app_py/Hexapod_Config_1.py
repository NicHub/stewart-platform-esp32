"""

S T E W A R T    P L A T F O R M    O N    E S P 3 2

Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""

import numpy as np
#
# ======== SERVO SETTINGS ========
#

NB_SERVOS = 6

#
# Servo calibration values.
# These values should not be used to restrict servo movements.
# Use MIN/MAX COORDINATES below for that.
#
SERVO_FULL_ANGULAR_RANGE = np.radians(180)
SERVO_HALF_ANGULAR_RANGE = SERVO_FULL_ANGULAR_RANGE / 2
SERVO_MIN_PWM = 183
SERVO_MAX_PWM = 467

#
# Offset values in µs to compensate for arm angle errors.
#
PW_OFFSET = (
    0,
    0,
    0,
    0,
    0,
    0,
)

#
# The gain in µs/rad
# Typical gain =~ 318 µs/rad =~ 5.6 µs/°
#
gain = (SERVO_MAX_PWM - SERVO_MIN_PWM) / (SERVO_FULL_ANGULAR_RANGE)

#
# calibration_t
#
# typedef struct
# {
#     double gain
#     int offset
# } calibration_t

#
# Calibration factors. These values take into account the fact
# that the odd and even arms are a reflection of each other.
# The calibration is linear:
# pulse width (µs) = gain (µs/rad) + offset (µs)
#
SERVO_CALIBRATION = (
    { "gain":-gain,  "offset": SERVO_MAX_PWM + PW_OFFSET[0] },
    { "gain":gain,   "offset": SERVO_MIN_PWM + PW_OFFSET[1] },
    { "gain":-gain,  "offset": SERVO_MAX_PWM + PW_OFFSET[2] },
    { "gain":gain,   "offset": SERVO_MIN_PWM + PW_OFFSET[3] },
    { "gain":-gain,  "offset": SERVO_MAX_PWM + PW_OFFSET[4] },
    { "gain":gain,   "offset": SERVO_MIN_PWM + PW_OFFSET[5] },
)

#
# ======== GEOMETRY SETTINGS ========
#

#
# Orientations of the servos arms relative to the X axis.
#
THETA_S = (
    np.radians(-60),
    np.radians(120),
    np.radians(180),
    np.radians(0),
    np.radians(60),
    np.radians(-120)
)

#
# MIN/MAX COORDINATES
# NOTE: The actual min and max for each DOF are interdependent. eg:
# If the platform is pitched by some amount, the roll min/max will be physically
# different than what’s defined here. These are just the absolute maximums under
# ideal conditions (eg: max for roll when pitch is zero).
#
HX_X_MIN = -24
HX_X_MAX = 24
HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2
HX_X_BAND = HX_X_MAX - HX_X_MIN

HX_Y_MIN = -24
HX_Y_MAX = 24
HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2
HX_Y_BAND = HX_Y_MAX - HX_Y_MIN

HX_Z_MIN = -12.0
HX_Z_MAX = 12.0
HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2
HX_Z_BAND = HX_Z_MAX - HX_Z_MIN

HX_A_MIN = np.radians(-12.0)
HX_A_MAX = np.radians(12.0)
HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2
HX_A_BAND = HX_A_MAX - HX_A_MIN

HX_B_MIN = np.radians(-12.0)
HX_B_MAX = np.radians(12.0)
HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2
HX_B_BAND = HX_B_MAX - HX_B_MIN

HX_C_MIN = np.radians(-43.0)
HX_C_MAX = np.radians(43.0)
HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2
HX_C_BAND = HX_C_MAX - HX_C_MIN

THETA_P = np.radians(48.4099) # Platform joint angle (radians) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes.
THETA_B = np.radians(22.9492) # Base Servo pinion angle (radians) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes.
P_RAD = 99.61 / 2          # Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
B_RAD = 153.99 / 2         # Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
ARM_LENGTH = 15.0          # Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
ROD_LENGTH = 140.0         # Push rod length (mm). Distance between pushrod ball joints (servo to platform).
Z_HOME = -132.943591247488 # Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints. Must Be fine tuned manualy or computed with a numerical solver.

#
# ======== ALGORITHM FOR SERVO ANGLE CALCULATIONS ========
#
ALGO = 3
