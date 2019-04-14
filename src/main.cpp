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

// Joystick
static ouilogique_Joystick joystick(X_PIN, Y_PIN, Z_PIN);

// Stewart Platform
static HexapodKinematics hk; // Stewart platform object.
static servo_t servo_angles[NB_SERVOS];
static Servo servos[NB_SERVOS]; // Servo objects.

/**
 *
 */
void printServoAngles()
{
    Serial.print("\nSERVO_ANGLES = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(degrees(servo_angles[sid].rad));
        Serial.print(" ");
    }
    Serial.print("\n");
}

/**
 *
 */
void testCalculations()
{
    hk.calcServoAngles(servo_angles, {0, 0, 0, 0, 0, 0});
    Serial.print("\n0, 0, 0, 0, 0, 0 ");
    printServoAngles();

    hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    Serial.print("\n0, 0, HEAVE_MAX, 0, 0, 0");
    printServoAngles();

    hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    Serial.print("\n0, 0, HEAVE_MIN, 0, 0, 0");
    printServoAngles();
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
        SET_LED;
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

    // Write to servos.
    for (int sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].writeMicroseconds(servo_angles[sid].pw);
    }

    CLEAR_LED;
}

/**
 *
 */
void setupServos()
{
    for (int sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].attach(SERVO_PINS[sid], SERVO_MIN_US, SERVO_MAX_US);
    }
    int8_t movOK = hk.home(servo_angles);
    updateServos(movOK);
    delay(500);
}

/**
 *
 */
void setupGPIO()
{
    pinMode(LED_BUILTIN, OUTPUT);
    CLEAR_LED;
}

/**
 *
 */
void shake()
{
    Serial.println("shake START");
    double shakeZ = HEAVE_MIN;
    int8_t movOK = -1;
    const uint32_t wait = 200;

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    for (int shake = 0; shake < 10; shake++)
    {
        delay(60);
        movOK = hk.calcServoAngles(servo_angles, {0, 0, shakeZ, 0, 0, 0});
        updateServos(movOK);
        shakeZ = -shakeZ;
    }

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MAX, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.calcServoAngles(servo_angles, {0, 0, HEAVE_MIN, 0, 0, 0});
    updateServos(movOK);

    delay(wait);
    movOK = hk.home(servo_angles);
    updateServos(movOK);

    Serial.println("shake DONE");
}

/**
 *
 */
void demoMovements2()
{
    int8_t movOK = -1;
    for (double cnt = 0; cnt < 50; cnt += 5)
    {
        Serial.print("cnt = ");
        Serial.println(cnt);
        movOK = hk.calcServoAngles(servo_angles, {cnt, 0, 0, 0, 0, 0});
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
        // sway
        {SWAY_MAX, 0, 0, 0, 0, 0},
        {SWAY_MIN, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // surge
        {0, SURGE_MAX, 0, 0, 0, 0},
        {0, SURGE_MIN, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // heave
        {0, 0, HEAVE_MAX, 0, 0, 0},
        {0, 0, HEAVE_MIN, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // pitch
        {0, 0, 0, PITCH_MAX, 0, 0},
        {0, 0, 0, PITCH_MIN, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // roll
        {0, 0, 0, 0, ROLL_MAX, 0},
        {0, 0, 0, 0, ROLL_MIN, 0},
        {0, 0, 0, 0, 0, 0},

        // yaw
        {0, 0, 0, 0, 0, YAW_MAX},
        {0, 0, 0, 0, 0, YAW_MIN},
        {0, 0, 0, 0, 0, 0}};

    int ccount = (int)sizeof(dval) / sizeof(dval[0]);

    int8_t movOK = -1;
    for (int cnt = 0; cnt < ccount; cnt++)
    {
        Serial.print("cnt = ");
        Serial.println(cnt);
        movOK = hk.calcServoAngles(servo_angles, {dval[cnt][0], dval[cnt][1], dval[cnt][2], dval[cnt][3], dval[cnt][4], dval[cnt][5]});
        updateServos(movOK);
        delay(1000);
    }
    Serial.println("demoMovements3 DONE");
}

/**
 *
 */
void demoMovements5(int nb_turn = 1)
{
    // Move in circles in the horizontal plane.

    Serial.println("demoMovements5 START");

    const int16_t nb_points = 90;
    const double radius = SWAY_MAX;
    const double angleInc = TWO_PI / nb_points;
    double angle = 0;
    double dval[nb_points][NB_SERVOS];
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
            movOK = hk.calcServoAngles(servo_angles, {dval[cnt][0], dval[cnt][1], dval[cnt][2], dval[cnt][3], dval[cnt][4], dval[cnt][5]});
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
    double joyX = joystick.getX();
    double joyY = joystick.getY();
    double joyZ = joystick.getZ();
    static double lastJoyX = 0;
    static double lastJoyY = 0;
    static double lastJoyZ = 0;
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

        // Set new limits.
        if (joyMode == 0)
            joystick.setLimits(SWAY_MIN, SWAY_MAX, SWAY_MID, SURGE_MIN, SURGE_MAX, SURGE_MID);
        else if (joyMode == 1)
            joystick.setLimits(YAW_MIN, YAW_MAX, YAW_MID, HEAVE_MIN, HEAVE_MAX, HEAVE_MID);
        else if (joyMode == 2)
            joystick.setLimits(PITCH_MIN, PITCH_MAX, PITCH_MID, ROLL_MIN, ROLL_MAX, ROLL_MID);

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
        movOK = hk.calcServoAngles(servo_angles, {joyX, joyY, 0, 0, 0, 0});
    else if (joyMode == 1)
        // Z, tiltZ
        movOK = hk.calcServoAngles(servo_angles, {0, 0, joyY, 0, 0, joyX});
    else if (joyMode == 2)
        // tilt X, tilt Y
        movOK = hk.calcServoAngles(servo_angles, {0, 0, 0, joyX, joyY, 0});

    updateServos(movOK);

    // Save last joystick values.
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
    joystick.setLimits(SWAY_MIN, SWAY_MAX, SWAY_MID, SURGE_MIN, SURGE_MAX, SURGE_MID);
    int8_t movOK = hk.home(servo_angles);
    updateServos(movOK);

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
        movOK = hk.calcServoAngles(servo_angles, {(double)sway, 0, 0, 0, 0, 0});
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
    setupGPIO();
    demoMovements5(2);
    shake();
    testCalculations();
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
