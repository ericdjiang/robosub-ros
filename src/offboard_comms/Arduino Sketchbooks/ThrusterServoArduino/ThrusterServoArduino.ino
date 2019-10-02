#include "Adafruit_PWMServoDriver.h"

#include "MultiplexedBasicESC.h"

#include "MultiplexedServo.h"

#include <ros.h>

#include <offboard_comms/ThrusterSpeeds.h>

#include <offboard_comms/ServoControl.h>

#include <Arduino.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define NUM_THRUSTERS 8

#define THRUSTER_TIMEOUT_MS 500

#define NUM_SERVO 8

// #define SERVO_TIMEOUT_MS 500

uint64_t last_cmd_ms_ts;

// uint64_t last_cmd_ms_sc;

int8_t thruster_speeds[NUM_THRUSTERS];

MultiplexedBasicESC *thrusters[NUM_THRUSTERS];

uint16_t servo_angle[NUM_SERVO];

// reusing ESC library code

MultiplexedBasicESC *servos[NUM_SERVO];

void thruster_speeds_callback(const offboard_comms::ThrusterSpeeds &ts_msg)

{

    //copy the contents of the speed message to the local array

    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));

    last_cmd_ms_ts = millis();

}

void servo_control_callback(const offboard_comms::ServoControl &sc_msg)

{

  //copy the contents of the angle message to the local array

  servo_angle_convert(sc_msg.speeds, servo_angle);

  // last_cmd_ms_sc = millis();

}

void servo_angle_convert(const uint16_t* s_angle, uint16_t* servo_angle) {

  for (int i = 0; i < 8; ++i) {

    if (s_angle[i] < 0 || s_angle[i] > 180) {

      servo_angle[i] = 0;

    }

    else {

      // servo goes from 1500 to 1900 in 0 to 180 degree, thruster goes from 1100 to 1900 in -128 to 128 power

      // so servo 0 to 180 would be mapped to 0-128 in thruster power value

      servo_angle[i] = map(s_angle[i], 0, 180, 0, 128);

    }

  }

}

ros::NodeHandle nh;

ros::Subscriber<offboard_comms::ThrusterSpeeds> ts_sub("/offboard/thruster_speeds", &thruster_speeds_callback);

ros::Subscriber<offboard_comms::ServoControl> sc_sub("/offboard/servo_control", &servo_control_callback);

void setup()

{

    nh.initNode();

    nh.subscribe(ts_sub);

    nh.subscribe(sc_sub);

    pwm_multiplexer.begin();

    for (int i = 0; i < NUM_THRUSTERS; ++i)

    {

        thrusters[i] = new MultiplexedBasicESC(&pwm_multiplexer, i);

        thrusters[i]->initialise(true);

    }

    for (int i = 0; i < NUM_SERVO; ++i)

    {

        servos[i] = new MultiplexedBasicESC(&pwm_multiplexer, i+8);

        servos[i]->initialise(false);

    }

    // Wait for motors to fully initialise

    delay(2000);

    last_cmd_ms_ts = millis();

//    last_cmd_ms_sc = millis();

}

void loop()

{

    // check if last version of data has timed out, if so, reset the speeds

    if (last_cmd_ms_ts + THRUSTER_TIMEOUT_MS < millis())

        memset(thruster_speeds, 0, sizeof(thruster_speeds));

    // if (last_cmd_ms_sc + SERVO_TIMEOUT_MS < millis())

    //     memset(servo_angle, servo_angle_convert(0), sizeof(servo_angle));

    for (int i = 0; i < NUM_THRUSTERS; ++i)

    {

        thrusters[i]->run(thruster_speeds[i], true);

    }

    for (int i = 0; i < NUM_SERVO; ++i)

    {

        servos[i]->run(servo_angle[i], false);

    }

    nh.spinOnce();

}
