#include <Arduino.h>
#include <Wire.h>
#include "MultiplexedServo.h"
#include "Adafruit_PWMServoDriver.h"

MultiplexedServo::MultiplexedServo(Adafruit_PWMServoDriver *_multiplexer) : multiplexer(_multiplexer)
{
  attached = false;
}

void MultiplexedServo::writeMicroseconds(int secs)
{
  if (attached)
  {
    int mappedPulse = map(secs, 1100, 1900, 1142, 1972);
    multiplexer->setPin(num, mappedPulse, false);
  }
}

void MultiplexedServo::attach(int _num)
{
  // make sure it isn't trying to run multiple pins
  if(attached)
    detach();
  attached = true;
  num = _num;
  multiplexer->setPWMFreq(250);
}

void MultiplexedServo::detach(){
  attached = false;
  multiplexer->setPin(num, 0, false);
}
