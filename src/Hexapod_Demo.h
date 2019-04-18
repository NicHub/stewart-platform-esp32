#ifndef __Hexapod_Demo_H__
#define __Hexapod_Demo_H__

#include <main.h>

class Hexapod_Demo
{
  public:
    Hexapod_Demo();
    void demoMov_MinMaxAllAxis();
    void demoMov_circles(uint8_t nb_turn);
    void demoMov_shake();
    void testNaN();
    void testCalculations();
    void testCalcSpeed(uint16_t nb_iter);
};

#endif
