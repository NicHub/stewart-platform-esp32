/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Based on
 * 6dof-stewduino
 * Copyright (C) 2018  Philippe Desrosiers
 * https://github.com/xoxota99/stewy
 *
 * Derived from the work of Daniel Waters
 * https://www.youtube.com/watch?v=1jrP3_1ML9M
 *
 * Modified by Nicolas Jeanmonod
 * ouilogique.com
 * March 2019
 *
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

#include <main.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

// Joystick
#define X_PIN 26
#define Y_PIN 12
#define Z_PIN 32
ouilogique_Joystick joystick(X_PIN, Y_PIN, Z_PIN);

// Stewart Platform
HexapodKinematics hk;           // Stewart platform object.
double servo_angles[NB_SERVOS]; // Servo setpoints in degrees, between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
Servo servos[NB_SERVOS];        // Servo objects.

float _toUs(int value)
{
    return map(value, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
}

/**
 * Set servo values to the angles represented by the setpoints in servo_angles[].
 * DOES: Apply trim values.
 * DOES: Automatically reverse signal for reversed servos.
 * DOES: Write signals to the physical servos.
 */
void updateServos(int8_t movOK)
{
    // Display error (movOK < 0)
    // and warnings (movOK > 0).
    if (movOK != 0)
    {
        Serial.print("\n");
        Serial.print(millis());
        Serial.print(" bad move - movOK = ");
        Serial.println(movOK, DEC);
        // If error, exit.
        if (movOK < 0)
        {
            return;
        }
    }

    Serial.print(".");

    static float sValues[NB_SERVOS];

    for (int sid = 0; sid < NB_SERVOS; sid++)
    {
        // servo_angles holds a value between SERVO_MIN_ANGLE and SERVO_MAX_ANGLE.
        // Apply reverse.
        float val = servo_angles[sid];
        if (SERVO_REVERSE[sid])
        {
            val = SERVO_MIN_ANGLE + (SERVO_MAX_ANGLE - val);
        }

        // Translate angle to pulse width.
        val = _toUs(val);

        if (val != sValues[sid])
        {
            sValues[sid] = val;
            servos[sid].writeMicroseconds((int)constrain(val + SERVO_TRIM[sid], SERVO_MIN_US, SERVO_MAX_US));
        }
    }
}

/**
 * Calculates and assigns values to servo_angles.
 * DOES: Ignore out-of-range values. These will generate a warning on the serial monitor.
 * DOES NOT: Apply servo trim values.
 * DOES NOT: Automatically reverse signal for reversed servos.
 * DOES NOT: digitally write a signal to any servo. Writing is done in updateServos();
 */
void setServo(int sid, int angle)
{
    int val = angle;
    if (val >= SERVO_MIN_ANGLE && val <= SERVO_MAX_ANGLE)
    {
        servo_angles[sid] = val;
        Serial.printf("setServo %d - %.2f degrees\n", sid, servo_angles[sid]);
    }
    else
    {
        Serial.printf("setServo: Invalid value '%d' specified for servo #%d. Valid range is %f to %f degrees.\n", val, sid, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    }
}

/**
 *
 */
void setupServos()
{
    for (int sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].attach(SERVO_PINS[sid], SERVO_MIN_US, SERVO_MAX_US);
        setServo(sid, SERVO_MID_ANGLE);
    }
    updateServos(true);
    delay(500);
}

/**
 *
 */
void shake()
{
    Serial.println("shake START");
    double shakeZ = MIN_HEAVE;
    int8_t movOK = -1;
    for (int shake = 0; shake < 10; shake++)
    {
        delay(60);
        movOK = hk.calcServoAngles(servo_angles, 0, 0, shakeZ, 0, 0, 0);
        updateServos(movOK);
        shakeZ = -shakeZ;
    }
    delay(200);
    Serial.println("shake DONE");
    Serial.println("home START");
    hk.home(servo_angles);
    updateServos(true);
    Serial.println("home DONE");
}

/**
 *
 */
void demoMovements1()
{
    for (int pos = SERVO_MIN_ANGLE; pos < SERVO_MAX_ANGLE; pos += 4)
    {
        for (int sid = 0; sid < NB_SERVOS; sid++)
        {
            setServo(sid, pos);
        }
        updateServos(true);
        delay(100);
    }
    // hk.home(servo_angles);
    Serial.println("demoMovements1 DONE");
}

/**
 *
 */
void demoMovements2()
{
    int8_t movOK = -1;
    for (size_t cnt = 0; cnt < 50; cnt += 5)
    {
        Serial.print("cnt = ");
        Serial.println(cnt);
        movOK = hk.calcServoAngles(servo_angles, cnt, 0, 0, 0, 0, 0);
        updateServos(movOK);
        delay(500);
    }
    Serial.println("demoMovements2 DONE");
}

/**
 *
 */
void demoMovements3()
{
    Serial.println("demoMovements3 START");
    const double dval[][NB_SERVOS] = {
        //sway
        {MAX_SWAY, 0, 0, 0, 0, 0},
        {MIN_SWAY, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        //surge
        {0, MAX_SURGE, 0, 0, 0, 0},
        {0, MIN_SURGE, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        //heave
        {0, 0, MAX_HEAVE, 0, 0, 0},
        {0, 0, MIN_HEAVE, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        //pitch
        {0, 0, 0, MAX_PITCH, 0, 0},
        {0, 0, 0, MIN_PITCH, 0, 0},
        {0, 0, 0, 0, 0, 0},

        //roll
        {0, 0, 0, 0, MAX_ROLL, 0},
        {0, 0, 0, 0, MIN_ROLL, 0},
        {0, 0, 0, 0, 0, 0},

        //yaw
        {0, 0, 0, 0, 0, MAX_YAW},
        {0, 0, 0, 0, 0, MIN_YAW},
        {0, 0, 0, 0, 0, 0}};

    int ccount = (int)sizeof(dval) / sizeof(dval[0]);

    int8_t movOK = -1;
    for (int cnt = 0; cnt < ccount; cnt++)
    {
        Serial.print("cnt = ");
        Serial.println(cnt);
        movOK = hk.calcServoAngles(servo_angles, dval[cnt][0], dval[cnt][1], dval[cnt][2], dval[cnt][3], dval[cnt][4], dval[cnt][5]);
        updateServos(movOK);
        delay(1000);
    }
    Serial.println("demoMovements3 DONE");
}

/**
 *
 */
void demoMovements4()
{
    setServo(0, SERVO_MIN_ANGLE);
    setServo(1, SERVO_MIN_ANGLE);
    setServo(2, SERVO_MIN_ANGLE);
    setServo(3, SERVO_MIN_ANGLE);
    setServo(4, SERVO_MAX_ANGLE + 1);
    setServo(5, SERVO_MAX_ANGLE + 1);
    updateServos(true);

    Serial.println("demoMovements4 DONE");
}

/**
 *
 */
void demoMovements5(int nb_turn = 1)
{
    // Move in circles in the horizontal plane.

    Serial.println("demoMovements5 START");

    const int16_t nb_points = 90;
    const double radius = MAX_SWAY;
    const double angleInc = TWO_PI / nb_points;
    double angle = 0;
    int dval[nb_points][NB_SERVOS];
    for (int16_t angleID = 0; angleID < nb_points; angleID++)
    {
        dval[angleID][0] = (int)(radius * sin(angle));
        dval[angleID][1] = (int)(radius * cos(angle));
        dval[angleID][2] = 0;
        dval[angleID][3] = 0;
        dval[angleID][4] = 0;
        dval[angleID][5] = 0;
        angle += angleInc;
    }
    int8_t movOK = -1;
    for (int turn = 0; turn < nb_turn; turn++)
    {
        for (int16_t cnt = 0; cnt < nb_points; cnt++)
        {
            movOK = hk.calcServoAngles(servo_angles, dval[cnt][0], dval[cnt][1], dval[cnt][2], dval[cnt][3], dval[cnt][4], dval[cnt][5]);
            updateServos(movOK);
            delay(8);
        }
    }
    Serial.println("demoMovements5 DONE");
}

/**
 *
 */
void joystickControl()
{
    int16_t joyX = joystick.getX();
    int16_t joyY = joystick.getY();
    int16_t joyZ = joystick.getZ();
    static int16_t lastJoyX = 0;
    static int16_t lastJoyY = 0;
    static int16_t lastJoyZ = 0;
    static uint8_t joyMode = 0;
    static const uint8_t nbJoyMode = 3;

    // Exit if joystick X, Y and Z did not change.
    bool joyStill;
    joyStill = ((joyX == lastJoyX) && (joyY == lastJoyY) && (joyZ == lastJoyZ));
    if (joyStill)
    {
        // Don’t check the joystick states too fast.
        // This is needed to remove noise.
        delay(10);
        return;
    }

    // Change joystick mode if button pressed.
    if (joyZ != 0)
    {
        joyMode = (joyMode + 1) % nbJoyMode;
        // Debounce.
        while (joystick.getZ())
        {
        }
        delay(250);
        return;
    }

    // Move according to joyMode.
    // TODO: Remember the previous joyMode positions when
    // changing joyMode instead of setting them to 0.
    int8_t movOK = -1;
    if (joyMode == 0)
        // X, Y
        movOK = hk.calcServoAngles(servo_angles, joyX, joyY, 0, 0, 0, 0);
    else if (joyMode == 1)
        // Z, tiltZ
        movOK = hk.calcServoAngles(servo_angles, 0, 0, joyY, 0, 0, joyX);
    else if (joyMode == 2)
        // tilt X, tilt Y
        movOK = hk.calcServoAngles(servo_angles, 0, 0, 0, joyX, joyY, 0);

    updateServos(movOK);

    // Record last joystick values.
    lastJoyX = joyX;
    lastJoyY = joyY;
    lastJoyZ = joyZ;

#if SEND_JOYSTICK_INFO_TO_SERIAL
    // Send joystick info to serial.
    Serial.print("\n\njoyX     = ");
    Serial.print(joyX);
    Serial.print(" | joyY     = ");
    Serial.print(joyY);
    Serial.print(" | joyZ     = ");
    Serial.println(joyZ);

    Serial.print("lastJoyX = ");
    Serial.print(lastJoyX);
    Serial.print(" | lastJoyY = ");
    Serial.print(lastJoyY);
    Serial.print(" | lastJoyZ = ");
    Serial.println(lastJoyZ);

    Serial.print("getRawX = ");
    Serial.print(joystick.getRawX());
    Serial.print(" | getRawY = ");
    Serial.println(joystick.getRawY());

    Serial.print("joyX == lastJoyX = ");
    Serial.println(joyX == lastJoyX);
    Serial.print("joyY == lastJoyY = ");
    Serial.println(joyY == lastJoyY);

    Serial.print("joyStill = ");
    Serial.println(joyStill);

    Serial.print("joyMode = ");
    Serial.println(joyMode);
#endif
}

/**
 *
 */
void setupJoystick()
{
    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(" | getRawValueMidX = ");
    Serial.print(joystick.getRawValueMidX());
    Serial.print(" | getRawValueMidY = ");
    Serial.println(joystick.getRawValueMidY());

    joystick.calibrate();
    delay(10);
    hk.home(servo_angles);
    updateServos(true);

    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(" | getRawValueMidX = ");
    Serial.print(joystick.getRawValueMidX());
    Serial.print(" | getRawValueMidY = ");
    Serial.println(joystick.getRawValueMidY());
}

/**
 *
 */
void serialControl()
{
    String message = "";
    if (serialRead(&message))
    {
        Serial.print("message = ");
        Serial.println(message);
        int sway = message.toInt();
        sway = sway % 30;
        Serial.print("sway = ");
        Serial.println(sway);

        int8_t movOK = -1;
        movOK = hk.calcServoAngles(servo_angles, sway, 0, 0, 0, 0, 0);
        updateServos(movOK);
    }
}

/**
 *
 */
void printJointAndServoAxisCoord()
{
    Serial.println("P_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(hk.P_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(hk.P_COORDS[sid][1]);
    }

    Serial.println("\nB_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(hk.B_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(hk.B_COORDS[sid][1]);
    }
}

/**
 *
 */
void setup()
{
    setupSerial();
    setupServos();
    setupJoystick();
    demoMovements5(5);
    shake();
}

/**
 *
 */
void loop()
{
#if ENABLE_JOYSTICK_READ
    joystickControl();
#endif

#if ENABLE_SERIAL_READ
    serialControl();
#endif
}
