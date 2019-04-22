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

#include <Hexapod_Serial.h>

extern angle_t servo_angles[NB_SERVOS];
extern Hexapod_Servo hx_servo;

/**
 *
 */
Hexapod_Serial::Hexapod_Serial()
{
}

/**
 * Prepares the input buffer to receive a new message
 * and tells the serial connected device it is ready for more.
 */
void Hexapod_Serial::ready()
{
    sofar = 0;              // Clear input buffer.
    Serial.println(F(">")); // Signal ready to receive input.
}

/**
 *
 */
void Hexapod_Serial::setupSerial()
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
    ready();
}

/**
 *
 **/
void Hexapod_Serial::G0(double newx, double newy, double newz,
                        double newa, double newb, double newc)
{
    px = newx;
    py = newy;
    pz = newz;
    pa = newa;
    pb = newb;
    pc = newc;

    static int8_t movOK;
    movOK = -1;
    movOK = hx_servo.calcServoAngles(
        {px, py, pz, radians(pa), radians(pb), radians(pc)},
        servo_angles);
    hx_servo.updateServos(movOK, 0UL);

    M114();
}

/**
 * Pause (dwell)
 * This will keep the axes unmoving for the period of time in seconds
 * specified by the P number. It is an error if the P number is negative.
 */
void Hexapod_Serial::G4P(double seconds)
{
    if (seconds >= 0.001)
        delay(seconds * 1000);
    else
        delayMicroseconds(seconds * 1000000);
}

/**
 *
 */
void Hexapod_Serial::G90()
{
    mode_abs = 1;
}

/**
 *
 */
void Hexapod_Serial::G91()
{
    mode_abs = 0;
}

/**
 * Print the current coordinates and absolute mode.
 */
void Hexapod_Serial::M114()
{
    output('X', px);
    output('Y', py);
    output('Z', pz);
    output('A', pa);
    output('B', pb);
    output('C', pc);
    Serial.println(mode_abs ? "ABS" : "REL");
}

/**
 * write a string followed by a double to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the double.
 */
void Hexapod_Serial::output(const char code, double val)
{
    Serial.print(code);
    Serial.print(val);
    Serial.print(" ");
}

/**
 * Read the input buffer and find any recognized commands.
 * One G or M command per line.
 */
void Hexapod_Serial::processCommand()
{
    int cmd = parseNumber('G', -1);
    switch (cmd)
    {

    // G0 [X/Y/Z/A/B/C]; - Rapid linear motion. X, Y, Z in mm | A, B, C in degrees.
    case 0:
    {
        G0(parseNumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
           parseNumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
           parseNumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
           parseNumber('A', (mode_abs ? pa : 0)) + (mode_abs ? 0 : pa),
           parseNumber('B', (mode_abs ? pb : 0)) + (mode_abs ? 0 : pb),
           parseNumber('C', (mode_abs ? pb : 0)) + (mode_abs ? 0 : pc));
        break;
    }

    // G4 P[seconds]; - Pause (dwell) in seconds.
    case 4:
        G4P(parseNumber('P', 0));
        break;

    // G90; - Go into absolute distance mode.
    case 90:
        G90();
        break;

    // G91; - Go into relative distance mode.
    case 91:
        G91();
        break;

    default:
        break;
    }

    cmd = parseNumber('M', -1);
    switch (cmd)
    {

    // M100 | Print help message.
    case 100:
        M100();
        break;

    // M114 | Print current coordinates.
    case 114:
        M114();
        break;

    default:
        break;
    }
}

/**
 * Look for character /code/ in the buffer and read the double that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
double Hexapod_Serial::parseNumber(const char code, double val)
{
    char *ptr = buffer; // start at the beginning of buffer
    while ((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer + sofar)
    { // walk to the end
        if (*ptr == code)
        {                         // if you find code on your walk,
            return atof(ptr + 1); // convert the digits that follow into a double and return it
        }
        ptr = strchr(ptr, ' ') + 1; // take a step from here to the letter after the next space
    }
    return val; // end reached, nothing found, return default val.
}

/**
 * display helpful information
 */
void Hexapod_Serial::M100()
{
    Serial.print(F("# HEXAPOD G-CODE PARSER #"));
    Serial.println(VERSION);
    Serial.println(F("## COMMANDS ##"));
    Serial.println(F("// G0 [X/Y/Z/A/B/C]; - Rapid linear motion. X, Y, Z in mm | A, B, C in degrees."));
    Serial.println(F("G4 P[seconds]; - Pause (dwell) in seconds"));
    Serial.println(F("G90; - absolute mode"));
    Serial.println(F("G91; - relative mode"));
    Serial.println(F("M100; - this help message"));
    Serial.println(F("M114; - report position"));
    Serial.println(F("All commands must end with a newline."));
}

/**
 *
 */
void Hexapod_Serial::serialRead()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();
        if (sofar < MAX_BUF - 1)
            buffer[sofar++] = c;
        if (c == '\n')
        {
            // End the buffer so string functions work right.
            buffer[sofar] = 0;
            Serial.print(F("\r\n"));
            processCommand();
            ready();
        }
    }
}
