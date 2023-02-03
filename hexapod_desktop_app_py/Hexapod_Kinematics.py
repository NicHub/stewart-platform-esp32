"""

S T E W A R T    P L A T F O R M    O N    E S P 3 2

Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
HEXAPOD_CONFIG = 1
if HEXAPOD_CONFIG == 1:
    import Hexapod_Config_1 as hc
import numpy as np
from pprint import pprint
from time import time
import datetime

SMALL_WIDTH = 7
LARGE_WIDTH = 17
START_DATE = datetime.datetime.now()

CPU_TIME_USED = 0
COUNTER = 0


#
# ======== PRECALCULATED GEOMETRY ==========
#

#
# There are three axes of symmetry (AXIS1, AXIS2, AXIS3). Looking down on the
# platform from above (along the Y axis), with 0 degrees being the X-positive line, and traveling
# in a CC direction, these are at 30 degrees, 120 degrees, and 240 degrees. All
# the polar coordinates of pivot points, servo centers, etc. are calculated based on
# an axis, and an offset angle (positive or negative theta) from the axis.

# NOTE: We make an assumption of mirror symmetry for AXIS3 along the Y axis.
# That is, AXIS1 is at (e.g.) 30 degrees, and AXIS3 will be at 120 degrees
# We account for this by negating the value of x-coordinates generated based
# on this axis later on. This is potentially messy, and should maybe be refactored.
#
AXIS1 = np.pi / 6  # 30 degrees.
AXIS2 = -np.pi / 2  # -90 degrees.
AXIS3 = AXIS1


#
# Orientations of the servos arms relative to the X axis.
#
COS_THETA_S = (
    np.cos(hc.THETA_S[0]),
    np.cos(hc.THETA_S[1]),
    np.cos(hc.THETA_S[2]),
    np.cos(hc.THETA_S[3]),
    np.cos(hc.THETA_S[4]),
    np.cos(hc.THETA_S[5]),
)


SIN_THETA_S = (
    np.sin(hc.THETA_S[0]),
    np.sin(hc.THETA_S[1]),
    np.sin(hc.THETA_S[2]),
    np.sin(hc.THETA_S[3]),
    np.sin(hc.THETA_S[4]),
    np.sin(hc.THETA_S[5]),
)

# For algorithm 2
M_THETA_S = (
    -hc.THETA_S[0],
    -hc.THETA_S[1],
    -hc.THETA_S[2],
    -hc.THETA_S[3],
    -hc.THETA_S[4],
    -hc.THETA_S[5],
)

sinD = (
    np.sin(M_THETA_S[0]),
    np.sin(M_THETA_S[1]),
    np.sin(M_THETA_S[2]),
    np.sin(M_THETA_S[3]),
    np.sin(M_THETA_S[4]),
    np.sin(M_THETA_S[5]),
)

cosD = (
    np.cos(M_THETA_S[0]),
    np.cos(M_THETA_S[1]),
    np.cos(M_THETA_S[2]),
    np.cos(M_THETA_S[3]),
    np.cos(M_THETA_S[4]),
    np.cos(M_THETA_S[5]),
)

#
# XY cartesian coordinates of the platform joints, based on the polar
# coordinates (platform radius P_RAD, radial axis AXIS[1|2|3], and offset THETA_P.
# These coordinates are in the plane of the platform itself.
#
P_COORDS = (
    (hc.P_RAD * np.cos(AXIS1 + hc.THETA_P), hc.P_RAD * np.sin(AXIS1 + hc.THETA_P)),
    (hc.P_RAD * np.cos(AXIS1 - hc.THETA_P), hc.P_RAD * np.sin(AXIS1 - hc.THETA_P)),
    (hc.P_RAD * np.cos(AXIS2 + hc.THETA_P), hc.P_RAD * np.sin(AXIS2 + hc.THETA_P)),
    (-hc.P_RAD * np.cos(AXIS2 + hc.THETA_P), hc.P_RAD * np.sin(AXIS2 + hc.THETA_P)),
    (-hc.P_RAD * np.cos(AXIS3 - hc.THETA_P), hc.P_RAD * np.sin(AXIS3 - hc.THETA_P)),
    (-hc.P_RAD * np.cos(AXIS3 + hc.THETA_P), hc.P_RAD * np.sin(AXIS3 + hc.THETA_P)),
)

#
# XY cartesian coordinates of the servo centers, based on the polar
# coordinates (base radius B_RAD, radial axis AXIS[1|2|3], and offset THETA_B.
# These coordinates are in the plane of the base itself.
#
B_COORDS = (
    (hc.B_RAD * np.cos(AXIS1 + hc.THETA_B), hc.B_RAD * np.sin(AXIS1 + hc.THETA_B)),
    (hc.B_RAD * np.cos(AXIS1 - hc.THETA_B), hc.B_RAD * np.sin(AXIS1 - hc.THETA_B)),
    (hc.B_RAD * np.cos(AXIS2 + hc.THETA_B), hc.B_RAD * np.sin(AXIS2 + hc.THETA_B)),
    (-hc.B_RAD * np.cos(AXIS2 + hc.THETA_B), hc.B_RAD * np.sin(AXIS2 + hc.THETA_B)),
    (-hc.B_RAD * np.cos(AXIS3 - hc.THETA_B), hc.B_RAD * np.sin(AXIS3 - hc.THETA_B)),
    (-hc.B_RAD * np.cos(AXIS3 + hc.THETA_B), hc.B_RAD * np.sin(AXIS3 + hc.THETA_B)),
)

#
# Square of the longest physically possible distance
# between servo pivot and platform joint.
#
BP2_MAX = (hc.ARM_LENGTH + hc.ROD_LENGTH) ** 2

#
# ARM^2, ARM^4, ROD^2, ROD^4,
#
ARM_LENGTH2 = hc.ARM_LENGTH**2
ARM_LENGTH4 = hc.ARM_LENGTH**4
ROD_LENGTH2 = hc.ROD_LENGTH**2
ROD_LENGTH4 = hc.ROD_LENGTH**4

#
# Square of the length of BP when the servo arm is perpendicular to BP.
#
BP2_PERP = ROD_LENGTH2 - ARM_LENGTH2


class Hexapod_Kinematics:
    """___"""

    def __init__(self):
        """___"""
        pass

    def mapDouble(self, x, in_min, in_max, out_min, out_max):
        """___"""
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def calcServoAngles(self, coord):
        """
        Calculation of the servo angles in radians, degrees and in microseconds (PWM)
        given the desired target platform coordinates.

        @param coord : the desired target platform coordinates.
        A struct containing X (surge), Y (sway), Z (heave) in mm
        and A (roll), B (pitch) and C (yaw) in radians.

        @param servo_angles : pointer to an array of struct containing
        the calculated servos angles in radians, degrees and in microseconds (PWM).

        @return
        Returns = 0 if OK
        Returns < 0 if Error
        """
        movOK = 0
        if hc.ALGO == 1:
            movOK, servo_angles = self.calcServoAnglesAlgo1(coord)
        elif hc.ALGO == 2:
            movOK, servo_angles = self.calcServoAnglesAlgo2(coord)
        elif hc.ALGO == 3:
            movOK, servo_angles = self.calcServoAnglesAlgo3(coord)

        return movOK, servo_angles

    def home(self):
        """
        HOME position. No translation, no rotation.
        """
        movOK, servo_angles = self.calcServoAngles(
            {"hx_x": 0, "hx_y": 0, "hx_z": 0, "hx_a": 0, "hx_b": 0, "hx_c": 0}
        )
        return movOK, servo_angles

    def calcServoAnglesAlgo3(self, coord):
        """___"""
        # Number of time the function was called.
        # nb_call = 0
        # ++nb_call;

        null_servo_angles = (
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
            {"rad": 0, "deg": 0, "pwm": 0, "pwm_us": 0},
        )
        new_servo_angles = null_servo_angles

        # Intermediate values, to avoid recalculating sin and cos.
        # (3 µs).
        cosA = np.cos(coord["hx_a"])
        cosB = np.cos(coord["hx_b"])
        cosC = np.cos(coord["hx_c"])
        sinA = np.sin(coord["hx_a"])
        sinB = np.sin(coord["hx_b"])
        sinC = np.sin(coord["hx_c"])

        # Assume everything will be OK.
        movOK = 0
        for sid in range(hc.NB_SERVOS):
            # Compute the new platform joint coordinates relative to servo pivot.
            # (~7 µs)
            BP_x = (
                P_COORDS[sid][0] * cosB * cosC
                + P_COORDS[sid][1] * (sinA * sinB * cosC - cosA * sinC)
                + coord["hx_x"]
                - B_COORDS[sid][0]
            )
            BP_y = (
                P_COORDS[sid][0] * cosB * sinC
                + P_COORDS[sid][1] * (sinA * sinB * sinC + cosA * cosC)
                + coord["hx_y"]
                - B_COORDS[sid][1]
            )
            BP_z = (
                -P_COORDS[sid][0] * sinB
                + P_COORDS[sid][1] * sinA * cosB
                + coord["hx_z"]
                - hc.Z_HOME
            )

            # print(f"{BP_x:20.1f}")
            # print(f"{cosA:20.5f}")
            # print(f'{coord["hx_a"]:20.5f}')

            a = COS_THETA_S[sid] * BP_x + SIN_THETA_S[sid] * BP_y
            b = -SIN_THETA_S[sid] * BP_x + COS_THETA_S[sid] * BP_y
            c = BP_z
            a2 = a**2
            a4 = a**4
            b2 = b**2
            b4 = b**4
            c2 = c**2
            c4 = c**4

            # Distance^2 between servo pivot (B) and platform joint (P).
            # Note that BP_x^2 + BP_y^2 + BP_z^2 = a^2 + b^2 + c^2, so
            # we can compare BP2 to BP2_MAX in the next test.
            BP2 = a2 + b2 + c2

            # Test if the new distance between servo pivot and platform joint
            # is longer than physically possible.
            # Abort computation of remaining angles if the current angle is not OK.
            # (~1 µs)
            if BP2 > BP2_MAX:
                movOK = -1
                break

            i1 = (
                -ARM_LENGTH4
                - ROD_LENGTH4
                - a4
                - b4
                - c4
                + 2
                * (
                    ARM_LENGTH2 * (ROD_LENGTH2 + a2 - b2 + c2)
                    + ROD_LENGTH2 * a2
                    + ROD_LENGTH2 * (b2 + c2)
                    - a2 * (b2 + c2)
                    - b2 * c2
                )
            )
            if i1 < 0:
                movOK = -5
                break
            i1 = np.sqrt(i1)
            i1 = (2 * hc.ARM_LENGTH * c - i1) / (
                ARM_LENGTH2 + 2 * hc.ARM_LENGTH * a - ROD_LENGTH2 + BP2
            )
            i1 = 2 * np.arctan(i1)
            new_servo_angles[sid]["rad"] = i1

            # Rotate the angle.
            # (~1 µs)
            new_servo_angles[sid]["rad"] += hc.SERVO_HALF_ANGULAR_RANGE

            # Convert radians to degrees.
            # (~2 µs)
            new_servo_angles[sid]["deg"] = np.rad2deg(new_servo_angles[sid]["rad"])

            # print(new_servo_angles[sid]["rad"])
            # print(hc.SERVO_CALIBRATION)
            # print(hc.SERVO_CALIBRATION[sid]["gain"])
            # return -1, null_servo_angles

            # Convert radians to PWM.
            # The calibration values take into account the fact
            # that the odd and even arms are a reflection of each other.
            # (~5 µs)
            new_servo_angles[sid]["pwm_us"] = (
                np.round(
                    hc.SERVO_CALIBRATION[sid]["gain"] * new_servo_angles[sid]["rad"]
                )
                + hc.SERVO_CALIBRATION[sid]["offset"]
            )

            # Check if the angle is in min/max.
            # Abort computation of remaining angles if the current angle is not OK.
            # (~1 µs)
            if new_servo_angles[sid]["pwm_us"] > hc.SERVO_MAX_PWM:
                movOK = -3
                break
            elif new_servo_angles[sid]["pwm_us"] < hc.SERVO_MIN_PWM:
                movOK = -4
                break

        # Update platform coordinates if there are no errors.
        # (~1 µs)
        if movOK != 0:
            new_servo_angles = null_servo_angles
        return movOK, new_servo_angles


def calcAndPrintResults(
    hk, coord={"hx_x": 0, "hx_y": 0, "hx_z": 0, "hx_a": 0, "hx_b": 0, "hx_c": 0}
):
    """___"""
    t1 = time()
    movOK, angles = hk.calcServoAngles(coord)
    t2 = time()

    global CPU_TIME_USED
    global COUNTER
    CPU_TIME_USED += (t2 - t1)
    COUNTER += 1

    ans = (
        f'{coord["hx_x"]:{SMALL_WIDTH}.1f}'
        f'{coord["hx_y"]:{SMALL_WIDTH}.1f}'
        f'{coord["hx_z"]:{SMALL_WIDTH}.1f}'
        f'{np.rad2deg(coord["hx_a"]):{SMALL_WIDTH}.1f}'
        f'{np.rad2deg(coord["hx_b"]):{SMALL_WIDTH}.1f}'
        f'{np.rad2deg(coord["hx_c"]):{SMALL_WIDTH}.1f}'
        f"{movOK:{SMALL_WIDTH}}"
    )
    if movOK == 0:
        ans += (
            f'{angles[0]["pwm_us"]:{LARGE_WIDTH}.0f}'
            f'{angles[1]["pwm_us"]:{LARGE_WIDTH}.0f}'
            f'{angles[2]["pwm_us"]:{LARGE_WIDTH}.0f}'
            f'{angles[3]["pwm_us"]:{LARGE_WIDTH}.0f}'
            f'{angles[4]["pwm_us"]:{LARGE_WIDTH}.0f}'
            f'{angles[5]["pwm_us"]:{LARGE_WIDTH}.0f}'
        )
    ans += "\n"
    return ans


if __name__ == "__main__":

    hk = Hexapod_Kinematics()
    ans = hk.home()
    # pprint(ans)
    # pprint(hc.HX_Y_MIN)
    shrink = 3
    nb_intervals = 1

    # HX_Xs = np.arange(hc.HX_X_MIN / shrink, hc.HX_X_MAX / shrink, ((hc.HX_X_MAX - hc.HX_X_MIN) / nb_intervals / shrink))
    # HX_Ys = np.arange(hc.HX_Y_MIN / shrink, hc.HX_Y_MAX / shrink, ((hc.HX_Y_MAX - hc.HX_Y_MIN) / nb_intervals / shrink))
    # HX_Zs = np.arange(hc.HX_Z_MIN / shrink, hc.HX_Z_MAX / shrink, ((hc.HX_Z_MAX - hc.HX_Z_MIN) / nb_intervals / shrink))
    # HX_As = np.arange(hc.HX_A_MIN / shrink, hc.HX_A_MAX / shrink, ((hc.HX_A_MAX - hc.HX_A_MIN) / nb_intervals / shrink))
    # HX_Bs = np.arange(hc.HX_B_MIN / shrink, hc.HX_B_MAX / shrink, ((hc.HX_B_MAX - hc.HX_B_MIN) / nb_intervals / shrink))
    # HX_Cs = np.arange(hc.HX_C_MIN / shrink, hc.HX_C_MAX / shrink, ((hc.HX_C_MAX - hc.HX_C_MIN) / nb_intervals / shrink))

    HX_Xs = [-8, 8]
    HX_Ys = [-8, 8]
    HX_Zs = [-4, 4]
    HX_As = [np.deg2rad(-12) / shrink, np.deg2rad(12) / shrink]
    HX_Bs = [np.deg2rad(-12) / shrink, np.deg2rad(12) / shrink]
    HX_Cs = [np.deg2rad(-43) / shrink, np.deg2rad(43) / shrink]

    ans = ""
    ans = "      X      Y      Z      A      B      C  movOK          ANGLE 1          ANGLE 2          ANGLE 3          ANGLE 4          ANGLE 5          ANGLE 6\n"

    ans = f"""
STEWART PLATFORM

COMPILATION DATE AND TIME
{START_DATE.strftime("%Y-%m-%d")}
{START_DATE.strftime("%H:%M:%S")}
HEXAPOD_CONFIG = {HEXAPOD_CONFIG}
ALGORITHM = {hc.ALGO}
LANGAGE = Python

      X      Y      Z      A      B      C  movOK          ANGLE 1          ANGLE 2          ANGLE 3          ANGLE 4          ANGLE 5          ANGLE 6
=======================================================================================================================================================
"""

    ans += calcAndPrintResults(
        hk, {"hx_x": 0, "hx_y": 0, "hx_z": 0, "hx_a": 0, "hx_b": 0, "hx_c": 0}
    )
    ans += calcAndPrintResults(
        hk, {"hx_x": 0, "hx_y": 0, "hx_z": hc.HX_Z_MAX, "hx_a": 0, "hx_b": 0, "hx_c": 0}
    )
    ans += calcAndPrintResults(
        hk, {"hx_x": 0, "hx_y": 0, "hx_z": hc.HX_Z_MIN, "hx_a": 0, "hx_b": 0, "hx_c": 0}
    )
    ans += "=======================================================================================================================================================\n"
    for hx_x in HX_Xs:
        for hx_y in HX_Ys:
            for hx_z in HX_Zs:
                for hx_a in HX_As:
                    for hx_b in HX_Bs:
                        for hx_c in HX_Cs:
                            coord = {
                                "hx_x": hx_x,
                                "hx_y": hx_y,
                                "hx_z": hx_z,
                                "hx_a": hx_a,
                                "hx_b": hx_b,
                                "hx_c": hx_c,
                            }
                            ans += calcAndPrintResults(hk, coord)

    CPU_TIME_USED *= 1E6
    ans += f"\nTotal time elapsed   (µs) = {CPU_TIME_USED:0.1f}"
    ans += f"\nTime per calculation (µs) = {CPU_TIME_USED / COUNTER:0.2f}"
    ans += f"\nCalculation count         = {COUNTER}"
    print(ans)

    filename = f"angles_with_config_{HEXAPOD_CONFIG}_py.txt"
    with open(file=filename, mode="wt", encoding="utf-8") as _f:
        _f.write(ans)
