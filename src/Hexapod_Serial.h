#ifndef __HEXAPOD_SERIAL_H__
#define __HEXAPOD_SERIAL_H__

#include <main.h>

#define MAX_BUF (64) // Serial buffer size

class Hexapod_Serial
{
  private:
    char buffer[MAX_BUF];          // where we store the message until we get a ';'
    int sofar = 0;                 // how much is in the buffer
    float px, py, pz, pa, pb, pc; // positions
    char mode_abs = 1;             // absolute mode?
    long line_number = 0;

    void ready();
    void pause(float seconds);
    void processCommand();
    float parseNumber(const char code, float val);
    void feedrate(float nfr);
    void position(float npx, float npy, float npz, float npa, float npb, float npc);
    void where();
    void output(const char code, float val);
    void help();
    void line(float newx, float newy, float newz, float newa, float newb, float newc);

  public:
    Hexapod_Serial();
    void setupSerial();
    void serialRead();
};

#endif
