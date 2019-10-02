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

void MultiplexedBasicESC::run(int power, bool thruster)
{
  if (thruster)
  {
    float pulse = map(power, -128, 128, 1100, 1900);
    Serial.println(pulse);
    servo->writeMicroseconds(pulse);
  }
  else
  {
    float pulse = map(power, 0, 180, 600, 3100);
    Serial.println(pulse);
    servo->writeMicroseconds(pulse);
  }
  
}

void MultiplexedBasicESC::initialise(bool thruster)
{
  servo->attach(num);
  if (thruster)
  {
    run(0, true);
  }
  else
  {
    run(90, false);
  }
  
}
