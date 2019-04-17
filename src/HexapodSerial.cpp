#include <HexapodSerial.h>

extern HexapodKinematics hk;            // Stewart platform object.
extern servo_t servo_angles[NB_SERVOS]; // Servo angles.

/**
 *
 */
void setupSerial()
{
    Serial.begin(BAUD_RATE);
    Serial.print("\n\n##########################");
    Serial.print("\nCOMPILATION DATE AND TIME:\n");
    Serial.print(__DATE__);
    Serial.print("\n");
    Serial.print(__TIME__);
    Serial.print("\nHEXAPOD_CONFIG = ");
    Serial.print(HEXAPOD_CONFIG);
    Serial.print("\n##########################\n\n");
}

/**
 *
 */
bool serialRead(String *message)
{
    int incomingByte = 0;
    bool gotMessage = false;

    if (Serial.available() > 0)
    {
        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingByte, DEC);

        *message = String(incomingByte);
        gotMessage = true;
    }
    return gotMessage;
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
        sway = sway % 10;
        Serial.print("sway = ");
        Serial.println(sway);

        int8_t movOK = -1;
        movOK = hk.calcServoAngles(servo_angles, {(double)sway, 0, 0, 0, 0, 0});
        updateServos(movOK);
    }
}
