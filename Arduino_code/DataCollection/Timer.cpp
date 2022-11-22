#include "Timer.h"

Timer::Timer()
{
  nextTimeout = 0;
}

void Timer::getTimer(int duration)
{
  nextTimeout = millis() + duration;
}

bool Timer::timerHasExpired()
{
  bool timerExpired = (millis() >= nextTimeout);
  return timerExpired;
}
