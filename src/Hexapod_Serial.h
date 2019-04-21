#ifndef __HEXAPOD_SERIAL_H__
#define __HEXAPOD_SERIAL_H__

#include <main.h>

#define MAX_BUF 64 // Serial buffer size

class Hexapod_Serial
{
  private:
    char buffer[MAX_BUF];         // where we store the message until we get a ';'
    int sofar = 0;                // how much is in the buffer
    double px, py, pz, pa, pb, pc; // positions
    char mode_abs = 1;            // absolute mode?
    long line_number = 0;

    void ready();
    void pause(double seconds);
    void processCommand();
    double parseNumber(const char code, double val);
    void feedrate(double nfr);
    void position(double npx, double npy, double npz, double npa, double npb, double npc);
    void where();
    void output(const char code, double val);
    void help();
    void line(double newx, double newy, double newz, double newa, double newb, double newc);

  public:
    Hexapod_Serial();
    void setupSerial();
    void serialRead();
};

#endif
