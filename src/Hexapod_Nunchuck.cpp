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

// The ATT pin is used to check if the Nunchuck is connected.
#define NUNCHUCK_ATT_PIN 35

/**
 * General stuff.
 */
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Average class. Used to smooth the signals of the accelerometer with a moving average.
 */
Average::Average()
{
}

/**
 *
 */
double Average::MovingAverage(double val, double in_low, double in_high, double out_low, double out_high)
{
    val = constrain(val, in_low, in_high);
    val = mapDouble(val, in_low, in_high, out_low, out_high);
    this->valHistory[this->index] = val;
    this->sum = 0;
    for (size_t cnt = 0; cnt < this->nbPoints; cnt++)
        this->sum += this->valHistory[cnt];
    this->index += 1;
    if (this->index >= this->nbPoints)
        this->index = 0;
    return this->sum / this->nbPoints;
}

/**
 * Hexapod_Nunchuck class.
 */
Hexapod_Nunchuck::Hexapod_Nunchuck() : Accessory()
{
}

/**
 *
 */
bool Hexapod_Nunchuck::connected()
{
    return digitalRead(NUNCHUCK_ATT_PIN);
}

/**
 *
 */
int Hexapod_Nunchuck::setupNunchuck()
{
    pinMode(NUNCHUCK_ATT_PIN, INPUT_PULLDOWN);
    // Abort if the Nunchuck is not connected
    if (!connected())
        return -1;
    begin();
    type = NUNCHUCK;
    readData();    // Read inputs and update maps
    printInputs(); // Print all inputs
    return 0;
}

/**
 *
 */
int Hexapod_Nunchuck::readNunchuck(nunchuck_t *nck)
{
    readData();
    nck->joy_x = mapDouble(getJoyX(), 0.0, 255.0, nckXmin, nckXmax);
    nck->joy_y = mapDouble(getJoyY(), 0.0, 255.0, nckYmin, nckYmax);
    nck->btn_c = getButtonC();
    nck->btn_z = getButtonZ();
    nck->acc_x = mapDouble(getAccelX(), 0.0, 255.0, nckXmin, nckXmax);
    nck->acc_y = mapDouble(getAccelY(), 0.0, 255.0, nckYmin, nckYmax);
    nck->acc_z = mapDouble(getAccelZ(), 0.0, 255.0, nckXmin, nckXmax);
    static const double limit = 45.0;
    static Average avPitchAngle;
    nck->pitch_angle = avPitchAngle.MovingAverage((double)getPitchAngle(), -limit, limit, nckAmin, nckAmax);
    static Average avRollAngle;
    nck->roll_angle = avRollAngle.MovingAverage((double)getRollAngle(), -limit, limit, nckBmin, nckBmax);
    return 0;
}

/**
 *
 */
void Hexapod_Nunchuck::nunchuckControl()
{
    // Don’t check the nunchuck states too fast.
    static long T1 = millis();
    if (millis() - T1 < 10)
        return;
    else
        T1 = millis();

    static nunchuck_t nck;
    readNunchuck(&nck);

    static int8_t joyMode = 0;
    static const int8_t nbJoyMode = 5;

    // Change nunchuck mode if button C or Z are pressed.
    // Button C increments joyMode.
    // Button Z decrements joyMode.
    if (nck.btn_c != 0 || nck.btn_z != 0)
    {
        if (nck.btn_c != 0)
            ++joyMode;
        else if (nck.btn_z != 0)
            --joyMode;
        if (joyMode == nbJoyMode)
            joyMode = 0;
        else if (joyMode < 0)
            joyMode = nbJoyMode - 1;
        Serial.print("JOYSTICK MODE = ");
        Serial.println(joyMode);

        // Set new limits.
        if (joyMode == 0)
        {
            nckXmin = HX_X_MIN;
            nckXmax = HX_X_MAX;
            nckYmin = HX_Y_MIN;
            nckYmax = HX_Y_MAX;
        }
        else if (joyMode == 1)
        {
            nckXmin = HX_C_MIN;
            nckXmax = HX_C_MAX;
            nckYmin = HX_Z_MIN;
            nckYmax = HX_Z_MAX;
        }
        else if (joyMode == 2)
        {
            nckXmin = HX_C_MIN;
            nckXmax = HX_C_MAX;
            nckYmin = HX_Z_MIN;
            nckYmax = HX_Z_MAX;
        }
        else if (joyMode == 3)
        {
            nckXmin = HX_A_MIN;
            nckXmax = HX_A_MAX;
            nckYmin = HX_B_MIN;
            nckYmax = HX_B_MAX;
        }
        else if (joyMode == 4)
        {
            nckAmin = HX_A_MIN;
            nckAmax = HX_A_MAX;
            nckBmin = HX_B_MIN;
            nckBmax = HX_B_MAX;
        }

        // Debounce.
        while (nck.btn_c || nck.btn_z)
        {
            readNunchuck(&nck);
            delay(20);
        }
        delay(200);

        return;
    }

    // Move according to joyMode.
    // Mode 0 to 3 use the joytick.
    // Mode 4 uses the joytick and the accelerometer.
    static int8_t movOK;
    movOK = -1;
    if (joyMode == 0)
        // X, Y
        movOK = hx_servo.calcServoAngles({nck.joy_x, nck.joy_y,
                                          0, 0, 0, 0},
                                         servo_angles);
    else if (joyMode == 1)
        // Z
        movOK = hx_servo.calcServoAngles({0, 0, nck.joy_y,
                                          0, 0, 0},
                                         servo_angles);
    else if (joyMode == 2)
        // tiltZ
        movOK = hx_servo.calcServoAngles({0, 0, 0,
                                          0, 0, nck.joy_x},
                                         servo_angles);
    else if (joyMode == 3)
        // tilt X, tilt Y
        movOK = hx_servo.calcServoAngles({0, 0, 0,
                                          nck.joy_x, nck.joy_y, 0},
                                         servo_angles);
    else if (joyMode == 4)
        // X, Y, tilt X, tilt Y
        movOK = hx_servo.calcServoAngles({nck.joy_x, nck.joy_y, 0,
                                          nck.pitch_angle, nck.roll_angle, 0},
                                         servo_angles);

    hx_servo.updateServos(movOK);

#if SEND_NUNCHUCK_INFO_TO_SERIAL
    // Send nunchuck info to serial.
    Serial.print("joy_x = ");
    Serial.print(nck.joy_x);

    Serial.print(" | joy_y = ");
    Serial.print(nck.joy_y);

    Serial.print(" | btn_c = ");
    Serial.print(nck.btn_c);

    Serial.print(" | pitch_angle = ");
    Serial.print(nck.pitch_angle);

    Serial.print(" | roll_angle = ");
    Serial.print(nck.roll_angle);

    Serial.print(" | joyMode = ");
    Serial.print(joyMode);

    Serial.print("\n");
#endif
}
