#include <Arduino.h>
#include <Wire.h>
#include "MultiplexedBasicESC.h"
#include "MultiplexedServo.h"

MultiplexedBasicESC::MultiplexedBasicESC(Adafruit_PWMServoDriver *_multiplexer, int _num) : num(_num)
{
  servo = new MultiplexedServo(_multiplexer);
}

MultiplexedBasicESC::~MultiplexedBasicESC()
{
  delete servo;
}

void MultiplexedBasicESC::run(int power)
{
  float pulse = map(power, -128, 128, 1100, 1900);
  Serial.println(pulse);
  servo->writeMicroseconds(pulse);
}

void MultiplexedBasicESC::initialise()
{
  servo->attach(num);
  run(0);
}
