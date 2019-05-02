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

#include <Hexapod_Nunchuck.h>
#include <Hexapod_Servo.h>
#include <Hexapod_GPIO.h>

extern angle_t servo_angles[NB_SERVOS];
extern Hexapod_Servo hx_servo;
extern Hexapod_GPIO hx_gpio;

/**
 *
 */
double Hexapod_Nunchuck::mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 *
 */
Hexapod_Nunchuck::Hexapod_Nunchuck() : Accessory()
{
}

/**
 *
 */
void Hexapod_Nunchuck::setupNunchuck()
{
    begin();
    type = NUNCHUCK;
    readData();    // Read inputs and update maps
    printInputs(); // Print all inputs
}

/**
 *
 */
void Hexapod_Nunchuck::nunchuckControl()
{
    static double joyX = 0;
    static double joyY = 0;
    static double joyZ = 0;
    static double lastJoyX = 0;
    static double lastJoyY = 0;
    static double lastJoyZ = 0;
    static uint8_t joyMode = 0;
    static const uint8_t nbJoyMode = 4;

    static double joyXmin = HX_X_MIN;
    static double joyXmax = HX_X_MAX;
    static double joyYmin = HX_Y_MIN;
    static double joyYmax = HX_Y_MAX;

    readData();
    joyX = mapDouble(getJoyX(), 0, 255, joyXmin, joyXmax);
    joyY = mapDouble(getJoyY(), 0, 255, joyYmin, joyYmax);
    joyZ = getButtonC();

    // Exit if nunchuck X, Y and Z did not change.
    static bool joyStill;
    joyStill = ((joyX == lastJoyX) && (joyY == lastJoyY) && (joyZ == lastJoyZ));
    if (joyStill)
    {
        // Don’t check the nunchuck states too fast.
        // This is needed to remove noise.
        delay(10);
        return;
    }

    // Change nunchuck mode if button pressed.
    if (joyZ != 0)
    {
        joyMode = (joyMode + 1) % nbJoyMode;
        Serial.print("JOYSTICK MODE = ");
        Serial.println(joyMode);

        // Set new limits.
        if (joyMode == 0)
        {
            joyXmin = HX_X_MIN;
            joyXmax = HX_X_MAX;
            joyYmin = HX_Y_MIN;
            joyYmax = HX_Y_MAX;
        }
        else if (joyMode == 1)
        {
            joyXmin = HX_C_MIN;
            joyXmax = HX_C_MAX;
            joyYmin = HX_Z_MIN;
            joyYmax = HX_Z_MAX;
        }
        else if (joyMode == 2)
        {
            joyXmin = HX_C_MIN;
            joyXmax = HX_C_MAX;
            joyYmin = HX_Z_MIN;
            joyYmax = HX_Z_MAX;
        }
        else if (joyMode == 3)
        {
            joyXmin = HX_A_MIN;
            joyXmax = HX_A_MAX;
            joyYmin = HX_B_MIN;
            joyYmax = HX_B_MAX;
        }

        // Debounce.
        while (getButtonC())
        {
            readData();
            delay(20);
        }
        delay(200);

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

    // Save last nunchuck values.
    lastJoyX = joyX;
    lastJoyY = joyY;
    lastJoyZ = joyZ;

#if SEND_NUNCHUCK_INFO_TO_SERIAL
    // Send nunchuck info to serial.
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

    Serial.print("joyStill = ");
    Serial.println(joyStill);

    Serial.print("joyMode = ");
    Serial.println(joyMode);
#endif
}
