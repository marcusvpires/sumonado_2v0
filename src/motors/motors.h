#ifndef _MOTORS_
#define _MOTORS_

#include <Arduino.h>

class Motors {
  public:
    void setup();
    void write(int va, int vb);
  private:
    const byte Ain1 = 13;
    const byte Ain2 = 12;
    const byte Bin1 = 25;
    const byte Bin2 = 26;
    const byte pwmA = 14;
    const byte pwmB = 27;
};

#endif