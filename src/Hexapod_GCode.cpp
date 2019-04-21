#include <Hexapod_GCode.h>

/**
 *
 */
Hexapod_GCode::Hexapod_GCode()
{
}

int Hexapod_GCode::test_atof()
{
    Serial.println("######### TEST atof ###########");
    double n;
    char buffer[256] = "3.1415926535";
    n = atof(buffer);
    Serial.printf("buffer = %f\n", n);
    return 0;
}

int Hexapod_GCode::test_strchr()
{
    Serial.println("######### TEST strchr ###########");
    char str[] = "This is a sample string";
    char *pch;
    Serial.printf("Looking for the 's' character in \"%s\"...\n", str);
    pch = strchr(str, 's');
    while (pch != NULL)
    {
        Serial.printf("found at %d\n", pch - str + 1);
        pch = strchr(pch + 1, 's');
    }
    return 0;
}
