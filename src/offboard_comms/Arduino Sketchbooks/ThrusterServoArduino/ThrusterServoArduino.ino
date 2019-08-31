#include "MultiplexedBasicESC.h"
#include "MultiplexedServo.h"
#include "Adafruit_PWMServoDriver.h"
#include <ros.h>
#include <offboard_comms/ThrusterServo.h>

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

#define NUM_THRUSTERS 8
#define THRUSTER_TIMEOUT_MS 500

uint64_t last_cmd_ms;

int8_t thruster_speeds[NUM_THRUSTERS];
MultiplexedBasicESC *thrusters[NUM_THRUSTERS];

void thruster_servo_callback(const offboard_comms::ThrusterServo ts_msg)
{
    //copy the contents of the speed message to the local array
    memcpy(thruster_speeds, ts_msg.thruster_speeds, sizeof(thruster_speeds));
    last_cmd_ms = millis();
}

ros::NodeHandle nh;
ros::Subscriber<offboard_comms::ThrusterServo> sub("offboard/thruster_servo", &thruster_servo_callback);

void setup()
{
    nh.initNode();
    nh.subscribe(sub);
    pwm_multiplexer.begin();
    Serial.begin(9600);

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
