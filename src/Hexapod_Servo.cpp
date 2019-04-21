#include <Hexapod_Servo.h>

extern angle_t servo_angles[NB_SERVOS];
extern Hexapod_GPIO hx_gpio;

/**
 *
 */
Hexapod_Servo::Hexapod_Servo()
{
}

/**
 *
 */
void Hexapod_Servo::setupServo()
{
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].attach(SERVO_PINS[sid], SERVO_MIN_US, SERVO_MAX_US);
    }
    int8_t movOK = home(servo_angles);
    this->updateServos(movOK);
    delay(500);
}

/**
 * Set servo values to the angles in servo_angles[].
 */
void Hexapod_Servo::updateServos(int8_t movOK, unsigned long safetyWait_ms)
{
    // Statistics of errors.
    static double nbMov = 0;
    static double nbMovNOK = 0;
    static double NOKrate = 0;
    nbMov++;

    if (movOK == 0)
    {
        // Do not update too quickly. This helps a little to prevent
        // the servos from going crazy. safetyWait_ms must be set to 0
        // when going on a smooth trajectory with a lot of points.
        static unsigned long T1;
        while ((millis() - T1) < safetyWait_ms)
        {
        }
        T1 = millis();

        // Write to servos.
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servos[sid].writeMicroseconds(servo_angles[sid].pw);
        }
    }
    else
    {
        // Error handling.
        hx_gpio.setBuiltInLED();
        nbMovNOK++;
        NOKrate = (nbMovNOK / nbMov) * (double)100.0;
        Serial.printf("%10lu", millis());
        Serial.printf(" | BAD MOVE | movOK = %d", movOK);
        Serial.printf(" | NB MOV = %10.0f", nbMov);
        Serial.printf(" | NOK rate = %4.1f %%", NOKrate);
        Serial.print("\n");
    }

    // Switch off LED.
    hx_gpio.clearBuiltInLEDDelayed(20);
}

/**
 *
 */
void Hexapod_Servo::printServoAngles()
{
    Serial.print("\nSERVO COORD        = ");
    Serial.print(getSway());
    Serial.print(" ");
    Serial.print(getSurge());
    Serial.print(" ");
    Serial.print(getHeave());
    Serial.print(" ");
    Serial.print(getPitch());
    Serial.print(" ");
    Serial.print(getRoll());
    Serial.print(" ");
    Serial.print(getYaw());

    Serial.print("\nSERVO_ANGLES (rad) = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles[sid].deg);
        Serial.print(" ");
    }

    Serial.print("\nSERVO_ANGLES (pwm) = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles[sid].pw);
        Serial.print(" ");
    }

    Serial.print("\nSERVO_ANGLES (debug) = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles[sid].debug);
        Serial.print(" ");
    }
    Serial.print("\n");
}

/**
 *
 */
void Hexapod_Servo::printJointAndServoAxisCoord()
{
    Serial.println("P_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(P_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(P_COORDS[sid][1]);
    }

    Serial.println("\nB_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(B_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(B_COORDS[sid][1]);
    }
}
