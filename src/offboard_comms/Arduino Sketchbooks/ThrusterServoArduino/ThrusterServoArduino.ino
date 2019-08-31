#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include "MultiplexedServo.h"
#include <ros.h>
#include <offboard_comms/ThrusterSpeeds.h>
#include <offboard_comms/ServoControl.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500

uint64_t last_cmd_ms;

int8_t thruster_speeds[NUM_THRUSTERS];
MultiplexedBasicESC *thrusters[NUM_THRUSTERS];

void thruster_speeds_callback(const offboard_comms::ThrusterSpeeds &ts_msg)
{
    //copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.speeds, sizeof(thruster_speeds));
    last_cmd_ms = millis();
}

void servo_control_callback(const offboard_comms::ServoControl &sc_msg)
{
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
        thrusters[i]->initialise();
    }

    // Wait for motors to fully initialise
    delay(2000);
    last_cmd_ms = millis();
}

void loop()
{
    // check if last version of data has timed out, if so, reset the speeds
    if (last_cmd_ms + THRUSTER_TIMEOUT_MS < millis())
        memset(thruster_speeds, 0, sizeof(thruster_speeds));

    for (int i = 0; i < NUM_THRUSTERS; ++i)
    {
        thrusters[i]->run(thruster_speeds[i]);
    }

    nh.spinOnce();
}
