#ifndef Servos_h
#define Servos_h

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"
#include <Servo.h>

class MultiplexedServo
{
private:
  int num;
  Adafruit_PWMServoDriver *multiplexer;

public:
  MultiplexedServo(Adafruit_PWMServoDriver *);
  void writeMicroseconds(int);
  void attach(int);
};

#endif
