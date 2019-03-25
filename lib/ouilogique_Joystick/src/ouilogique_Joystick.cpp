#include "ouilogique_Joystick.h"

ouilogique_Joystick::ouilogique_Joystick(uint8_t pinX, uint8_t pinY, uint8_t pinZ)
{
    this->pinX = pinX;
    this->pinY = pinY;
    this->pinZ = pinZ;
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinZ, INPUT_PULLUP);
}

int16_t ouilogique_Joystick::getRawX()
{
    int16_t rawValue = analogRead(this->pinX);
    return rawValue;
}

int16_t ouilogique_Joystick::getRawY()
{
    int16_t rawValue = analogRead(this->pinY);
    return rawValue;
}

int16_t ouilogique_Joystick::getX()
{
    int16_t rawValue = analogRead(this->pinX);
    int16_t value;
    if(rawValue >= DEADBANDMINX && rawValue <= DEADBANDMAXX)
        value = OUT_MID;
    else if(rawValue >= RAWVALUEMIDX)
        value = map(rawValue, RAWVALUEMAXX, RAWVALUEMIDX, OUT_MIN, OUT_MID);
    else if(rawValue < RAWVALUEMIDX)
        value = map(rawValue, RAWVALUEMIDX, RAWVALUEMINX, OUT_MID, OUT_MAX);
    return value;
}

int16_t ouilogique_Joystick::getY()
{
    int16_t rawValue = analogRead(this->pinY);
    int16_t value;
    if(rawValue >= DEADBANDMINY && rawValue <= DEADBANDMAXY)
        value = OUT_MID;
    else if(rawValue >= RAWVALUEMIDY)
        value = map(rawValue, RAWVALUEMAXY, RAWVALUEMIDY, OUT_MIN, OUT_MID);
    else if(rawValue < RAWVALUEMIDY)
        value = map(rawValue, RAWVALUEMIDY, RAWVALUEMINY, OUT_MID, OUT_MAX);
    return value;
}

bool ouilogique_Joystick::getZ()
{
    bool value = !digitalRead(this->pinZ);
    return value;
}
