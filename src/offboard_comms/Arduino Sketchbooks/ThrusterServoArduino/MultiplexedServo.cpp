#include <Arduino.h>
#include <Wire.h>
#include "MultiplexedServo.h"
#include "Adafruit_PWMServoDriver.h"

MultiplexedServo::MultiplexedServo(Adafruit_PWMServoDriver *_multiplexer) : multiplexer(_multiplexer)
{
}

void MultiplexedServo::writeMicroseconds(int secs)
{
  int mappedPulse = map(secs, 1100, 1900, 1142, 1972);
  multiplexer->setPWM(num, 0, mappedPulse);
}

void MultiplexedServo::attach(int _num)
{
  num = _num;
  multiplexer->setPWMFreq(250);
}
