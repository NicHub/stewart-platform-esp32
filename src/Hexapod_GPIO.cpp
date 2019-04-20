#include <Hexapod_GPIO.h>

/**
 *
 */
void Hexapod_GPIO::setupGPIO()
{
    pinMode(LED_BUILTIN, OUTPUT);
    clearBuiltInLED();
}

/**
 *
 */
void Hexapod_GPIO::setBuiltInLED()
{
    digitalWrite(LED_BUILTIN, LOW);
    builtInLEDState = true;
}

/**
 *
 */
void Hexapod_GPIO::clearBuiltInLED()
{
    digitalWrite(LED_BUILTIN, HIGH);
    builtInLEDState = false;
}

/**
 *
 */
void Hexapod_GPIO::clearBuiltInLEDDelayed(unsigned long dt)
{
    static unsigned long T1 = millis();
    if (millis() - T1 > dt)
    {
        clearBuiltInLED();
        T1 = millis();
    }
}

/**
 *
 */
void Hexapod_GPIO::blinkBuitInLED(uint8_t nb_iter, unsigned long tON, unsigned long tOFF)
{
    for (uint8_t cnt = 0; cnt < nb_iter; cnt++)
    {
        setBuiltInLED();
        delay(tON);
        clearBuiltInLED();
        delay(tOFF);
    }
}
