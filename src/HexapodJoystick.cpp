#include <HexapodJoystick.h>

ouilogique_Joystick joystick(X_PIN, Y_PIN, Z_PIN);
extern HexapodKinematics hk;            // Stewart platform object.
extern servo_t servo_angles[NB_SERVOS]; // Servo angles.
extern Servo servos[NB_SERVOS];         // Servo objects.

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
void joystickControl()
{
    static double joyX = 0;
    static double joyY = 0;
    static double joyZ = 0;
    static double lastJoyX = 0;
    static double lastJoyY = 0;
    static double lastJoyZ = 0;
    static uint8_t joyMode = 0;
    static const uint8_t nbJoyMode = 3;

    joyX = joystick.getX();
    joyY = joystick.getY();
    joyZ = joystick.getZ();

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
            joystick.setLimits(SWAY_MIN, SWAY_MAX, SWAY_MID, SURGE_MIN, SURGE_MAX, SURGE_MID);
        else if (joyMode == 1)
            joystick.setLimits(YAW_MIN, YAW_MAX, YAW_MID, HEAVE_MIN, HEAVE_MAX, HEAVE_MID);
        else if (joyMode == 2)
            joystick.setLimits(PITCH_MIN, PITCH_MAX, PITCH_MID, ROLL_MIN, ROLL_MAX, ROLL_MID);

        // Debounce.
        while (joystick.getZ())
        {
        }

        // Blink.
        for (uint8_t cnt = 0; cnt < 3; cnt++)
        {
            SET_LED;
            delay(20);
            CLEAR_LED;
            delay(100);
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
