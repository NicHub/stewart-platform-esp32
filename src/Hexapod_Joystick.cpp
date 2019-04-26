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

#include <Hexapod_Joystick.h>
#include <Hexapod_Servo.h>
#include <Hexapod_GPIO.h>

extern angle_t servo_angles[NB_SERVOS];
extern Hexapod_Servo hx_servo;
extern Hexapod_GPIO hx_gpio;

/**
 *
 */
Hexapod_Joystick::Hexapod_Joystick(
    uint8_t pinX, uint8_t pinY, uint8_t pinZ) : ouilogique_Joystick(pinX, pinY, pinZ)
{
}

/**
 *
 */
void Hexapod_Joystick::setupJoystick()
{
    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(" | getRawValueMidX = ");
    Serial.print(getRawValueMidX());
    Serial.print(" | getRawValueMidY = ");
    Serial.println(getRawValueMidY());

    calibrate();
    delay(10);
    setLimits(HX_X_MIN, HX_X_MAX, HX_X_MID, HX_Y_MIN, HX_Y_MAX, HX_Y_MID);
    int8_t movOK = hx_servo.home(servo_angles);
    hx_servo.updateServos(movOK);

    Serial.print("millis = ");
    Serial.print(millis());
    Serial.print(" | getRawValueMidX = ");
    Serial.print(getRawValueMidX());
    Serial.print(" | getRawValueMidY = ");
    Serial.println(getRawValueMidY());
}

/**
 *
 */
void Hexapod_Joystick::joystickControl()
{
    static double joyX = 0;
    static double joyY = 0;
    static double joyZ = 0;
    static double lastJoyX = 0;
    static double lastJoyY = 0;
    static double lastJoyZ = 0;
    static uint8_t joyMode = 0;
    static const uint8_t nbJoyMode = 4;

    joyX = getX();
    joyY = getY();
    joyZ = getZ();

    // Exit if joystick X, Y and Z did not change.
    static bool joyStill;
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
        Serial.print("JOYSTICK MODE = ");
        Serial.println(joyMode);

        // Set new limits.
        if (joyMode == 0)
            setLimits(HX_X_MIN, HX_X_MAX, HX_X_MID, HX_Y_MIN, HX_Y_MAX, HX_Y_MID);
        else if (joyMode == 1)
            setLimits(HX_C_MIN, HX_C_MAX, HX_C_MID, HX_Z_MIN, HX_Z_MAX, HX_Z_MID);
        else if (joyMode == 2)
            setLimits(HX_C_MIN, HX_C_MAX, HX_C_MID, HX_Z_MIN, HX_Z_MAX, HX_Z_MID);
        else if (joyMode == 3)
            setLimits(HX_A_MIN, HX_A_MAX, HX_A_MID, HX_B_MIN, HX_B_MAX, HX_B_MID);

        // Blink built in LED (joyMode + 1) times.
        hx_gpio.blinkBuitInLED(joyMode + 1, 150, 75);

        // Debounce.
        while (getZ())
        {
        }

        return;
    }

    // Move according to joyMode.
    // TODO: Remember the previous joyMode positions when
    // changing joyMode instead of setting them to 0.
    static int8_t movOK;
    movOK = -1;
    if (joyMode == 0)
        // X, Y
        movOK = hx_servo.calcServoAngles({joyX, joyY, 0, 0, 0, 0}, servo_angles);
    else if (joyMode == 1)
        // Z
        movOK = hx_servo.calcServoAngles({0, 0, joyY, 0, 0, 0}, servo_angles);
    else if (joyMode == 2)
        // tiltZ
        movOK = hx_servo.calcServoAngles({0, 0, 0, 0, 0, joyX}, servo_angles);
    else if (joyMode == 3)
        // tilt X, tilt Y
        movOK = hx_servo.calcServoAngles({0, 0, 0, joyX, joyY, 0}, servo_angles);

    hx_servo.updateServos(movOK);

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
    Serial.print(getRawX());
    Serial.print(" | getRawY = ");
    Serial.println(getRawY());

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
