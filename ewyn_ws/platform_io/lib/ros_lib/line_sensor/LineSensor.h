#ifndef _ROS_line_sensor_LineSensor_h
#define _ROS_line_sensor_LineSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace line_sensor
{

  class LineSensor : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int32_t count;

    LineSensor():
      header(),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->count = u_count.real;
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return "line_sensor/LineSensor"; };
    const char * getMD5(){ return "77b9c1ba0b019866302b213c3d49f362"; };

  };

}
#endif