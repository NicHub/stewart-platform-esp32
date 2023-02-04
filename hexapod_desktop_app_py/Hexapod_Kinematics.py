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

##################

Works on
MicroPython v1.19.1 on 2022-06-18; ESP32 module with ESP32

Doesn’t work on
MicroPython v1.16-200-g1b87e1793 on 2021-08-12; LOLIN_S2_MINI with ESP32-S2FN4R2
(no f-string support)

| SYSTEM                       | TIME PER CALCULATION |
| ---------------------------- | -------------------: |
| MicroPython v1.19.1 on ESP32 |            ~ 4000 µs |
| C++ on ESP32                 |             ~ 250 µs |
| Python on Apple M1 Pro       |              ~ 12 µs |
| C++ on Apple M1 Pro          |               ~ 2 µs |

"""
HEXAPOD_CONFIG = 1
if HEXAPOD_CONFIG == 1:
    import Hexapod_Config_1 as hc
import math
import time

SMALL_WIDTH = 7
LARGE_WIDTH = 17
START_DATE = time.time()

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
AXIS1 = math.pi / 6  # 30 degrees.
AXIS2 = -math.pi / 2  # -90 degrees.
AXIS3 = AXIS1


#
# Orientations of the servos arms relative to the X axis.
#
COS_THETA_S = (
    math.cos(hc.THETA_S[0]),
    math.cos(hc.THETA_S[1]),
    math.cos(hc.THETA_S[2]),
    math.cos(hc.THETA_S[3]),
    math.cos(hc.THETA_S[4]),
    math.cos(hc.THETA_S[5]),
)


SIN_THETA_S = (
    math.sin(hc.THETA_S[0]),
    math.sin(hc.THETA_S[1]),
    math.sin(hc.THETA_S[2]),
    math.sin(hc.THETA_S[3]),
    math.sin(hc.THETA_S[4]),
    math.sin(hc.THETA_S[5]),
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
    math.sin(M_THETA_S[0]),
    math.sin(M_THETA_S[1]),
    math.sin(M_THETA_S[2]),
    math.sin(M_THETA_S[3]),
    math.sin(M_THETA_S[4]),
    math.sin(M_THETA_S[5]),
)

cosD = (
    math.cos(M_THETA_S[0]),
    math.cos(M_THETA_S[1]),
    math.cos(M_THETA_S[2]),
    math.cos(M_THETA_S[3]),
    math.cos(M_THETA_S[4]),
    math.cos(M_THETA_S[5]),
)

#
# XY cartesian coordinates of the platform joints, based on the polar
# coordinates (platform radius P_RAD, radial axis AXIS[1|2|3], and offset THETA_P.
# These coordinates are in the plane of the platform itself.
#
P_COORDS = (
    (hc.P_RAD * math.cos(AXIS1 + hc.THETA_P), hc.P_RAD * math.sin(AXIS1 + hc.THETA_P)),
    (hc.P_RAD * math.cos(AXIS1 - hc.THETA_P), hc.P_RAD * math.sin(AXIS1 - hc.THETA_P)),
    (hc.P_RAD * math.cos(AXIS2 + hc.THETA_P), hc.P_RAD * math.sin(AXIS2 + hc.THETA_P)),
    (-hc.P_RAD * math.cos(AXIS2 + hc.THETA_P), hc.P_RAD * math.sin(AXIS2 + hc.THETA_P)),
    (-hc.P_RAD * math.cos(AXIS3 - hc.THETA_P), hc.P_RAD * math.sin(AXIS3 - hc.THETA_P)),
    (-hc.P_RAD * math.cos(AXIS3 + hc.THETA_P), hc.P_RAD * math.sin(AXIS3 + hc.THETA_P)),
)

#
# XY cartesian coordinates of the servo centers, based on the polar
# coordinates (base radius B_RAD, radial axis AXIS[1|2|3], and offset THETA_B.
# These coordinates are in the plane of the base itself.
#
B_COORDS = (
    (hc.B_RAD * math.cos(AXIS1 + hc.THETA_B), hc.B_RAD * math.sin(AXIS1 + hc.THETA_B)),
    (hc.B_RAD * math.cos(AXIS1 - hc.THETA_B), hc.B_RAD * math.sin(AXIS1 - hc.THETA_B)),
    (hc.B_RAD * math.cos(AXIS2 + hc.THETA_B), hc.B_RAD * math.sin(AXIS2 + hc.THETA_B)),
    (-hc.B_RAD * math.cos(AXIS2 + hc.THETA_B), hc.B_RAD * math.sin(AXIS2 + hc.THETA_B)),
    (-hc.B_RAD * math.cos(AXIS3 - hc.THETA_B), hc.B_RAD * math.sin(AXIS3 - hc.THETA_B)),
    (-hc.B_RAD * math.cos(AXIS3 + hc.THETA_B), hc.B_RAD * math.sin(AXIS3 + hc.THETA_B)),
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

        # Intermediate values, to avoid recalculating sin math cos.
        # (3 µs).
        cosA = math.cos(coord["hx_a"])
        cosB = math.cos(coord["hx_b"])
        cosC = math.cos(coord["hx_c"])
        sinA = math.sin(coord["hx_a"])
        sinB = math.sin(coord["hx_b"])
        sinC = math.sin(coord["hx_c"])

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
            i1 = math.sqrt(i1)
            i1 = (2 * hc.ARM_LENGTH * c - i1) / (
                ARM_LENGTH2 + 2 * hc.ARM_LENGTH * a - ROD_LENGTH2 + BP2
            )
            i1 = 2 * math.atan(i1)
            new_servo_angles[sid]["rad"] = i1

            # Rotate the angle.
            # (~1 µs)
            new_servo_angles[sid]["rad"] += hc.SERVO_HALF_ANGULAR_RANGE

            # Convert radians to degrees.
            # (~2 µs)
            new_servo_angles[sid]["deg"] = math.degrees(new_servo_angles[sid]["rad"])

            # print(new_servo_angles[sid]["rad"])
            # print(hc.SERVO_CALIBRATION)
            # print(hc.SERVO_CALIBRATION[sid]["gain"])
            # return -1, null_servo_angles

            # Convert radians to PWM.
            # The calibration values take into account the fact
            # that the odd and even arms are a reflection of each other.
            # (~5 µs)
            new_servo_angles[sid]["pwm_us"] = (
                round(hc.SERVO_CALIBRATION[sid]["gain"] * new_servo_angles[sid]["rad"])
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

    try:
        t1 = time.ticks_us()
    except AttributeError:
        t1 = time.time() * 1e6
    movOK, angles = hk.calcServoAngles(coord)
    try:
        t2 = time.ticks_us()
    except AttributeError:
        t2 = time.time() * 1e6

    global CPU_TIME_USED
    global COUNTER
    CPU_TIME_USED += t2 - t1
    COUNTER += 1

    ans = f'{coord["hx_x"]:{SMALL_WIDTH}.1f}'
    ans += f'{coord["hx_y"]:{SMALL_WIDTH}.1f}'
    ans += f'{coord["hx_z"]:{SMALL_WIDTH}.1f}'
    ans += f'{math.degrees(coord["hx_a"]):{SMALL_WIDTH}.1f}'
    ans += f'{math.degrees(coord["hx_b"]):{SMALL_WIDTH}.1f}'
    ans += f'{math.degrees(coord["hx_c"]):{SMALL_WIDTH}.1f}'
    ans += f"{movOK:{SMALL_WIDTH}}"

    if movOK == 0:
        ans += f'{angles[0]["pwm_us"]:{LARGE_WIDTH}.0f}'
        ans += f'{angles[1]["pwm_us"]:{LARGE_WIDTH}.0f}'
        ans += f'{angles[2]["pwm_us"]:{LARGE_WIDTH}.0f}'
        ans += f'{angles[3]["pwm_us"]:{LARGE_WIDTH}.0f}'
        ans += f'{angles[4]["pwm_us"]:{LARGE_WIDTH}.0f}'
        ans += f'{angles[5]["pwm_us"]:{LARGE_WIDTH}.0f}'

    ans += "\n"
    return ans


if __name__ == "__main__":

    hk = Hexapod_Kinematics()
    ans = hk.home()
    # pprint(ans)
    # pprint(hc.HX_Y_MIN)
    shrink = 3
    nb_intervals = 1

    HX_Xs = [-8, 8]
    HX_Ys = [-8, 8]
    HX_Zs = [-4, 4]
    HX_As = [math.radians(-12) / shrink, math.radians(12) / shrink]
    HX_Bs = [math.radians(-12) / shrink, math.radians(12) / shrink]
    HX_Cs = [math.radians(-43) / shrink, math.radians(43) / shrink]

    ans = ""
    ans += "STEWART PLATFORM\n"
    # ans += "COMPILATION DATE AND TIME\n"
    # ans += f"{START_DATE}\n"
    # ans += f"{time.time()}\n"
    # ans += '{START_DATE.strftime("%H:%M:%S")}\n'
    ans += f"HEXAPOD_CONFIG = {HEXAPOD_CONFIG}\n"
    ans += f"ALGORITHM = {hc.ALGO}\n"
    ans += f"LANGAGE = Python\n"
    ans += "\n"
    ans += "      X      Y      Z      A      B      C  movOK          ANGLE 1          ANGLE 2          ANGLE 3          ANGLE 4          ANGLE 5          ANGLE 6\n"
    ans += "=======================================================================================================================================================\n"

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

    ans += f"\nTotal time elapsed   (µs) = {CPU_TIME_USED:0.1f}"
    ans += f"\nTime per calculation (µs) = {CPU_TIME_USED / COUNTER:0.2f}"
    ans += f"\nCalculation count         = {COUNTER}"
    print(ans)

    filename = f"angles_with_config_{HEXAPOD_CONFIG}_py.txt"
    with open(file=filename, mode="wt", encoding="utf-8") as _f:
        _f.write(ans)
