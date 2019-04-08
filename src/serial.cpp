/**
 * serial.cpp
 *
 * ouilogique.com
 * april 2019
 *
 */

#include <main.h>

void setupSerial()
{
    Serial.begin(BAUD_RATE);
    Serial.print("\n\n##########################");
    Serial.print("\nCOMPILATION DATE AND TIME:\n");
    Serial.print(__DATE__);
    Serial.print("\n");
    Serial.print(__TIME__);
    Serial.print("\n##########################\n\n");
}

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
