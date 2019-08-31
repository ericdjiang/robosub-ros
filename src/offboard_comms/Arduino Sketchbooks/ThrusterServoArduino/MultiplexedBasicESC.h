#ifndef BasicESC_h
#define BasicESC_h

#include <Arduino.h>
#include "Motor.h"
#include "MultiplexedServo.h"
#include <Servo.h>

class MultiplexedBasicESC
{
private:
  int num;
  MultiplexedServo *servo;

public:
  MultiplexedBasicESC(Adafruit_PWMServoDriver *, int);
  ~MultiplexedBasicESC();
  void initialise();
  void run(int);
};

#endif
