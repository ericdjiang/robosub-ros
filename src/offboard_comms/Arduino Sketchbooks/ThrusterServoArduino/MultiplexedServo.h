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
  bool attached;

public:
  MultiplexedServo(Adafruit_PWMServoDriver *);
  ~MultiplexedServo();
  void writeMicroseconds(int);
  void attach(int);
  void detach();
};

#endif
