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

#include <Hexapod_GPIO.h>

/**
 *
 */
void Hexapod_GPIO::setupGPIO()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DEBUG_PIN_0, OUTPUT);
    pinMode(DEBUG_PIN_1, OUTPUT);
    pinMode(DEBUG_PIN_2, OUTPUT);
    pinMode(DEBUG_PIN_3, OUTPUT);
    pinMode(DEBUG_PIN_4, OUTPUT);
    pinMode(DEBUG_PIN_5, OUTPUT);
    clearBuiltInLED();
    chaseDebug();
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

/**
 *
 */
void Hexapod_GPIO::chaseDebug()
{
#define wait 10
    for (size_t _i = 0; _i < 6; _i++)
    {
        digitalWrite(DEBUG_PIN_0, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_1, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_2, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_3, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_4, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_5, HIGH);
        delay(wait);
        digitalWrite(DEBUG_PIN_4, LOW);
        delay(wait);
        digitalWrite(DEBUG_PIN_3, LOW);
        delay(wait);
        digitalWrite(DEBUG_PIN_2, LOW);
        delay(wait);
        digitalWrite(DEBUG_PIN_1, LOW);
        delay(wait);
        digitalWrite(DEBUG_PIN_0, LOW);
        delay(wait);
    }
}
