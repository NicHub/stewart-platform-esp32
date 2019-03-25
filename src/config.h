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

#include <Arduino.h>

#ifndef __STU_CONFIG_H__
#define __STU_CONFIG_H__

#define ENABLE_SERIAL_COMMANDS    //Comment out, to omit Command shell code.
// #define ENABLE_NUNCHUCK           //Comment out, to omit Nunchuck code.
// #define ENABLE_TOUCHSCREEN        //Comment out, to omit Touchscreen code.

/*
  Comment out, to disable Servos. Servos can get hot if you don't set
  them to a "safe" value. Commenting this line out allows to test things that
  are not servo-related, without killing the battery, browning out the USB,
  or overheating the servos.
*/
// #define ENABLE_SERVOS

#define LOG_LEVEL       Logger::DEBUG

//Which servos are reversed. 1 = reversed, 0 = normal.
// const int SERVO_REVERSE[6] = {0, 1, 0, 1, 0, 1};
const int SERVO_REVERSE[6] = {0, 0, 1, 0, 1, 0}; // Attention le premier moteur n’est pas du même type que les autres c’est pourquoi il est inversé.

#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     100
const int SERVO_MID_ANGLE = SERVO_MIN_ANGLE + (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 2;

#define SERVO_MIN_US        800
#define SERVO_MAX_US        2000
const int SERVO_MID_US = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) / 2;

const int SERVO_TRIM[] = {  //trim values, in microseconds, AFTER reversing
  0,
  20,
  0,
  135,
  0,
  120
};

const int SERVO_PINS[] = {  //pin numbers for each servo signal.
  13,
  15,
  27,
  14,
  33,
  25
};


typedef struct xy_coordf {
  float x;  //-1.0 to 1.0
  float y;   //-1.0 to 1.0
} xy_coordf;

const xy_coordf DEFAULT_SETPOINT = {0,0};

/*
  NOTE: The actual "min" and "max" for each DOF are interdependent. eg:
  If the platform is pitched by some amount, the roll min/max will be physically
  different than what's defined here. These are just the absolute maximums under
  ideal conditions (eg: max for roll when pitch is zero).
*/

#define MIN_PITCH  -20
#define MAX_PITCH  23
const int PITCH_BAND = MAX_PITCH - MIN_PITCH;

#define MIN_ROLL   -23
#define MAX_ROLL   20
const int ROLL_BAND = MAX_ROLL - MIN_ROLL;

#define MIN_YAW   -69
#define MAX_YAW   69
const int YAW_BAND = MAX_YAW - MIN_YAW;

#define MIN_SWAY   -55
#define MAX_SWAY   55
const int SWAY_BAND = MAX_SWAY - MIN_SWAY;

#define MIN_SURGE   -70
#define MAX_SURGE   55
const int SURGE_BAND = MAX_SURGE - MIN_SURGE;

#define MIN_HEAVE   -22
#define MAX_HEAVE   25
const int HEAVE_BAND = MAX_HEAVE - MIN_HEAVE;

/*
* ======== Nunchuck Settings ==========
*/

#ifdef ENABLE_NUNCHUCK

Blinker blinker = Blinker::attach(LED_BUILTIN, true, 150, 150);

/*
  Define a deadband for the nunchuck joystick. If we're in the deadband during CONTROL mode, the platform will move to the HOME position.
*/
xy_coordf deadBand = {2,2};     //sort of using xy_coordf for the wrong thing here...

/*
  Specifies the maximum time between button clicks that are interpreted as a
  double-click. If the time between clicks exceeds this value, the clicks are
  interpreted as single clicks.
*/
#define NUNCHUCK_DBLCLICK_THRESHOLD_MS  500

/*
  Delay between movements of the Setpoint, in SQUARE mode.
*/
#define SQUARE_DELAY_MS 1000

/*
  Epsilon values for floating point comparison of Nunchuck tilt values.
*/
// #define TILT_EPSILON 0.001

//Different "modes" for the platform.
enum Mode {
  SETPOINT,   // setpoint position is in the middle of the platform.
  CONTROL,    // X/Y position of the wiichuck directly controls the position of the platform.
  CIRCLE,     // setpoint position moves in a circle. Direction (CW / CCW) is controlled by the "direction" field.
  EIGHT,      // setpoint position moves in a figure eight. Direction (CW / CCW) is controlled by the "direction" field
  SQUARE      // setpoint cycles through corners of a square (manually controlled by the Z button).
};

char const * modeStrings[] = {
  "SETPOINT",
  "CONTROL",
  "CIRCLE",
  "EIGHT",
  "SQUARE"
};

enum ControlSubMode {
  PITCH_ROLL, // Joystick controls the angle of the platform.
  HEAVE_YAW,  // Joystick Y axis controls the up-down movement of the platform, X axis controls the rotation of the platform.
  SWAY_SURGE  // Joystick X axis controls sway, Y axis contrls surge.
};

char const * subModeStrings[] = {
  "PITCH_ROLL",
  "HEAVE_YAW",
  "SWAY_SURGE"
};

enum Direction {
  CW,
  CCW
};

char const * directionStrings[] = {
  "CW",
  "CCW"
};

#define DEFAULT_MODE        SETPOINT
#define DEFAULT_SUB_MODE    PITCH_ROLL
#define SETPOINT_MOVE_RATE  0.001F

Mode mode = DEFAULT_MODE;
Direction dir = CW;
ControlSubMode controlSubMode = DEFAULT_SUB_MODE;

#define DEFAULT_SPEED 0.2F;

float sp_speed = DEFAULT_SPEED;  //speed (between 0.0 and 1.0) of the movement of the setpoint, for modes that support automatic setpoint movement.
float sp_radius;                 //radius, for modes that need a radius. For CIRCLE, this is the plain old radius. For EIGHT, this is the farthest distance from the center of the plate.
#endif    //ENABLE_NUNCHUCK


/*
* ======== Platform / Servo Settings ==========
*/

// Geometry of the platform.

#define THETA_P_DEG     45.25     //Platform joint angle (degrees) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes
#define THETA_B_DEG     24.5      //Base Servo pinion angle (degrees) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes
#define THETA_P         (THETA_P_DEG * PI / 180)  //Theta P, in radians
#define THETA_B         (THETA_B_DEG * PI / 180)  //Theta B, in radians
#define P_RAD           50        //Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
#define B_RAD           80.2      //Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
#define ARM_LENGTH      25        //Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
#define ROD_LENGTH      155       //Push rod length (mm). Distance between pushrod ball joints (servo to platform).
#define Z_HOME          148       //Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints.

/*
  If defined, the IK algorithm will "slam" values to min or max when it encounters
  an asymptotic condition. That is, if the solution requires that the servo (e.g.) extend
  beyond its physical limits, it will set the servo angle to MAX.

  If NOT defined, the IK algorithm will simply refuse to modify ANY servo endpoints
  when it encounters an asymptotic condition.
*/
// #define SLAM

/*
  Prescalar to the output of the platform IK solution for each servo.
  NOTE: Even with aggro, the solution will never fall outside the range of
  [SERVO_ANGLE_MIN .. SERVO_ANGLE_MAX]
*/
#define AGGRO       1.5F

/*
   There are three axes of symmetry (AXIS1, AXIS2, AXIS3). Looking down on the
   platform from above (along the Y axis), with 0 degrees being the X-positive line, and traveling
   in a CC direction, these are at 30 degrees, 120 degrees, and 240 degrees. All
   the polar coordinates of pivot points, servo centers, etc. are calculated based on
   an axis, and an offset angle (positive or negative theta) from the axis.
 */

#define AXIS1       (PI / 6)  //30 degrees.
#define AXIS2       (-PI / 2) //-90 degrees.
/*
   NOTE: We make an assumption of mirror symmetry for AXIS3 along the Y axis.
   That is, AXIS1 is at (e.g.) 30 degrees, and AXIS3 will be at 120 degrees
   We account for this by negating the value of x-coordinates generated based
   on this axis later on. This is potentially messy, and should maybe be refactored.
 */
#define AXIS3             AXIS1

/*
   Absolute angle that the servo arm plane of rotation is at (degrees), from the world-X axis.
 */
const double THETA_S_DEG[6] = {
  -60,
  120,
  180,
  0,
  60,
  -120
};

const double THETA_S[6] = {     //Servo arm angle (radians)
  radians(THETA_S_DEG[0]),
  radians(THETA_S_DEG[1]),
  radians(THETA_S_DEG[2]),
  radians(THETA_S_DEG[3]),
  radians(THETA_S_DEG[4]),
  radians(THETA_S_DEG[5])
};

/*
   XY cartesian coordinates of the platform joints, based on the polar
   coordinates (platform radius P_RAD, radial axis AXIS[1|2\3], and offset THETA_P.
   These coordinates are in the plane of the platform itself.
 */
const double P_COORDS[6][2] = {
  {P_RAD * cos(AXIS1 + THETA_P), P_RAD * sin(AXIS1 + THETA_P)},
  {P_RAD * cos(AXIS1 - THETA_P), P_RAD * sin(AXIS1 - THETA_P)},
  {P_RAD * cos(AXIS2 + THETA_P), P_RAD * sin(AXIS2 + THETA_P)},
  { -P_RAD * cos(AXIS2 + THETA_P), P_RAD * sin(AXIS2 + THETA_P)},
  { -P_RAD * cos(AXIS3 - THETA_P), P_RAD * sin(AXIS3 - THETA_P)},
  { -P_RAD * cos(AXIS3 + THETA_P), P_RAD * sin(AXIS3 + THETA_P)}
};

/*
   XY cartesian coordinates of the servo centers, based on the polar
   coordinates (base radius B_RAD, radial axis AXIS[1|2\3], and offset THETA_B.
   These coordinates are in the plane of the base itself.
 */
const double B_COORDS[6][2] = {
  {B_RAD * cos(AXIS1 + THETA_B), B_RAD * sin(AXIS1 + THETA_B)},
  {B_RAD * cos(AXIS1 - THETA_B), B_RAD * sin(AXIS1 - THETA_B)},
  {B_RAD * cos(AXIS2 + THETA_B), B_RAD * sin(AXIS2 + THETA_B)},
  { -B_RAD * cos(AXIS2 + THETA_B), B_RAD * sin(AXIS2 + THETA_B)},
  { -B_RAD * cos(AXIS3 - THETA_B), B_RAD * sin(AXIS3 - THETA_B)},
  { -B_RAD * cos(AXIS3 + THETA_B), B_RAD * sin(AXIS3 + THETA_B)}
};

/*
* ============ Touchscreen config ============
*/
#ifdef ENABLE_TOUCHSCREEN

#define XP A7  // YELLOW / XRT. can be a digital pin.
#define XM A6  // WHITE / XLE. must be an analog pin, use "An" notation!

#define YP A8  // RED / YLO. must be an analog pin, use "An" notation!
#define YM A9  // BLACK / YUP. can be a digital pin.

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 711 ohms across the X plate
#define TS_OHMS 711 //resistance between X+ and X-

//The Adafruit touchscreen library returns raw values from the ADC (between 0-1024).
//Here, we adjust for our specific touchscreen part. (In this case, https://www.digikey.com/product-detail/en/nkk-switches/FTAS00-12.1AN-4/360-3520-ND/6823699)

//Min / max values of X and Y.
#define TS_MIN_X              1
#define TS_MAX_X              950       //1023
const int TS_WIDTH = TS_MAX_X-TS_MIN_X;

#define TS_MIN_Y              100
#define TS_MAX_Y              930       //1023
const int TS_HEIGHT = TS_MAX_Y-TS_MIN_Y;

/*
  Time (in millis) between the touch sensor "losing" the ball, and the platform
  getting a signal to go to the "home" position. Until this time has passed, the
  platform will stay in it's last position.
*/
#define LOST_BALL_TIMEOUT     250

double setpointX=TS_MIN_X+(TS_WIDTH/2);
double inputX;
double outputX;

double setpointY=TS_MIN_Y+(TS_HEIGHT/2);
double inputY;
double outputY;

//Specify the links and initial tuning parameters
double PX=3, IX=0, DX=0;
double PY=1, IY=0, DY=0;

#define ROLL_PID_SAMPLE_TIME 10
#define ROLL_PID_LIMIT_MIN -1024
#define ROLL_PID_LIMIT_MAX 1024

#define PITCH_PID_SAMPLE_TIME 10
#define PITCH_PID_LIMIT_MIN -1024
#define PITCH_PID_LIMIT_MAX 1024

#endif    //ENABLE_TOUCHSCREEN

#endif    //__STU_CONFIG_H__
