//=======================================================================
// The class for the timer functions

#ifndef Timer_h
#define Timer_h
#include "Arduino.h"

class Timer
{
  public:
    void getTimer(int duration);
    bool timerHasExpired();
    Timer();

  private:
    unsigned long nextTimeout;
};

#endif
