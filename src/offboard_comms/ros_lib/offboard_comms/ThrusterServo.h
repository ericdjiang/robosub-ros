#ifndef _ROS_offboard_comms_ThrusterServo_h
#define _ROS_offboard_comms_ThrusterServo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace offboard_comms
{

  class ThrusterServo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      int8_t thruster_speeds[8];

    ThrusterServo():
      header(),
      thruster_speeds()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_thruster_speedsi;
      u_thruster_speedsi.real = this->thruster_speeds[i];
      *(outbuffer + offset + 0) = (u_thruster_speedsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->thruster_speeds[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_thruster_speedsi;
      u_thruster_speedsi.base = 0;
      u_thruster_speedsi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->thruster_speeds[i] = u_thruster_speedsi.real;
      offset += sizeof(this->thruster_speeds[i]);
      }
     return offset;
    }

    const char * getType(){ return "offboard_comms/ThrusterServo"; };
    const char * getMD5(){ return "4eb0e6fd4ae7e912e89fc1baecc22da4"; };

  };

}
#endif