#include <Hexapod_Servo.h>

extern Hexapod_Kinematics hk;            // Stewart platform object.
extern servo_t servo_angles[NB_SERVOS]; // Servo angles.
extern Servo servos[NB_SERVOS];         // Servo objects.


/**
 *
 */
void setupServos()
{
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].attach(SERVO_PINS[sid], SERVO_MIN_US, SERVO_MAX_US);
    }
    int8_t movOK = hk.home(servo_angles);
    updateServos(movOK);
    delay(500);
}

/**
 * Set servo values to the angles in servo_angles[].
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
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        servos[sid].writeMicroseconds(servo_angles[sid].pw);
    }

    CLEAR_LED;
}

/**
 *
 */
void printServoAngles()
{
    Serial.print("\nSERVO COORD        = ");
    Serial.print(hk.getSway());
    Serial.print(" ");
    Serial.print(hk.getSurge());
    Serial.print(" ");
    Serial.print(hk.getHeave());
    Serial.print(" ");
    Serial.print(hk.getPitch());
    Serial.print(" ");
    Serial.print(hk.getRoll());
    Serial.print(" ");
    Serial.print(hk.getYaw());

    Serial.print("\nSERVO_ANGLES (rad) = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(degrees(servo_angles[sid].rad));
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
