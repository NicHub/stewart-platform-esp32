/**
 * Code adapted from
 * https://github.com/MarginallyClever/GcodeCNCDemo/tree/master/GcodeCNCDemo4AxisCNCShield
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
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void Hexapod_Serial::ready()
{
    sofar = 0;              // clear input buffer
    Serial.println(F(">")); // signal ready to receive input
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
 * pause (dwell)
 * This will keep the axes unmoving for the period of time in seconds
 * specified by the P number. It is an error if the P number is negative.
 */
void Hexapod_Serial::pause(float seconds)
{
    if (seconds >= 0.001)
        delay(seconds * 1000);
    else
        delayMicroseconds(seconds * 1000000);
}

/**
 *
 **/
void Hexapod_Serial::line(float newx, float newy, float newz, float newa, float newb, float newc)
{
    position(newx, newy, newz, newa, newb, newc);

    static int8_t movOK;
    movOK = -1;
    movOK = hx_servo.calcServoAngles(
        {px, py, pz, radians(pa), radians(pb), radians(pc)},
        servo_angles);
    hx_servo.updateServos(movOK);

    where();
}

/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void Hexapod_Serial::processCommand()
{
    int cmd = parseNumber('G', -1);
    switch (cmd)
    {
    case 0:
    {
        line(parseNumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
             parseNumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
             parseNumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz),
             parseNumber('A', (mode_abs ? pa : 0)) + (mode_abs ? 0 : pa),
             parseNumber('B', (mode_abs ? pb : 0)) + (mode_abs ? 0 : pb),
             parseNumber('C', (mode_abs ? pb : 0)) + (mode_abs ? 0 : pc));
        break;
    }
    case 4:
        pause(parseNumber('P', 0));
        break; // dwell in seconds
    case 90:
        mode_abs = 1;
        break; // absolute mode
    case 91:
        mode_abs = 0;
        break; // relative mode
    case 92:   // set logical position
        position(
            parseNumber('X', 0),
            parseNumber('Y', 0),
            parseNumber('Z', 0),
            parseNumber('A', 0),
            parseNumber('B', 0),
            parseNumber('C', 0));
        break;
    default:
        break;
    }

    cmd = parseNumber('M', -1);
    switch (cmd)
    {
    case 100:
        help();
        break;
    case 114:
        where();
        break;
    default:
        break;
    }
}

/**
 * print the current position, feedrate, and absolute mode.
 */
void Hexapod_Serial::where()
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
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float Hexapod_Serial::parseNumber(const char code, float val)
{
    char *ptr = buffer; // start at the beginning of buffer
    while ((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer + sofar)
    { // walk to the end
        if (*ptr == code)
        {                         // if you find code on your walk,
            return atof(ptr + 1); // convert the digits that follow into a float and return it
        }
        ptr = strchr(ptr, ' ') + 1; // take a step from here to the letter after the next space
    }
    return val; // end reached, nothing found, return default val.
}

/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void Hexapod_Serial::position(float npx, float npy, float npz, float npa, float npb, float npc)
{
    px = npx;
    py = npy;
    pz = npz;
    pa = npa;
    pb = npb;
    pc = npc;
}

/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void Hexapod_Serial::output(const char code, float val)
{
    Serial.print(code);
    Serial.print(val);
    Serial.print(" ");
}

/**
 * display helpful information
 */
void Hexapod_Serial::help()
{
    Serial.print(F("GcodeCNCDemo6AxisV2 "));
    Serial.println(VERSION);
    Serial.println(F("Commands:"));
    Serial.println(F("G00/G01 [X/Y/Z/E(steps)] [F(feedrate)]; - linear move"));
    Serial.println(F("G04 P[seconds]; - delay"));
    Serial.println(F("G90; - absolute mode"));
    Serial.println(F("G91; - relative mode"));
    Serial.println(F("G92 [X/Y/Z/E(steps)]; - change logical position"));
    Serial.println(F("M18; - disable motors"));
    Serial.println(F("M100; - this help message"));
    Serial.println(F("M114; - report position and feedrate"));
    Serial.println(F("All commands must end with a newline."));
}

/**
 *
 */
void Hexapod_Serial::serialRead()
{
    // listen for serial commands
    while (Serial.available() > 0)
    {                           // if something is available
        char c = Serial.read(); // get it
        // Serial.print(c);        // repeat it back so I know you got the message
        if (sofar < MAX_BUF - 1)
            buffer[sofar++] = c; // store it
        if (c == '\n')
        {
            // entire message received
            buffer[sofar] = 0;       // end the buffer so string functions work right
            Serial.print(F("\r\n")); // echo a return character for humans
            processCommand();        // do something with the command
            ready();
        }
    }
}
