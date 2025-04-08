#ifndef _ROS_sonicservo_Servo_h
#define _ROS_sonicservo_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sonicservo
{

  class Servo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _channel_name_type;
      _channel_name_type channel_name;
      typedef uint8_t _channel_id_type;
      _channel_id_type channel_id;
      typedef uint32_t _safe_us_type;
      _safe_us_type safe_us;
      typedef uint32_t _default_us_type;
      _default_us_type default_us;
      typedef uint32_t _pulsewidth_us_type;
      _pulsewidth_us_type pulsewidth_us;
      typedef uint32_t _max_us_type;
      _max_us_type max_us;
      typedef uint32_t _min_us_type;
      _min_us_type min_us;
      typedef uint32_t _speed_type;
      _speed_type speed;

    Servo():
      header(),
      channel_name(""),
      channel_id(0),
      safe_us(0),
      default_us(0),
      pulsewidth_us(0),
      max_us(0),
      min_us(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_channel_name = strlen(this->channel_name);
      varToArr(outbuffer + offset, length_channel_name);
      offset += 4;
      memcpy(outbuffer + offset, this->channel_name, length_channel_name);
      offset += length_channel_name;
      *(outbuffer + offset + 0) = (this->channel_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channel_id);
      *(outbuffer + offset + 0) = (this->safe_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->safe_us >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->safe_us >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->safe_us >> (8 * 3)) & 0xFF;
      offset += sizeof(this->safe_us);
      *(outbuffer + offset + 0) = (this->default_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->default_us >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->default_us >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->default_us >> (8 * 3)) & 0xFF;
      offset += sizeof(this->default_us);
      *(outbuffer + offset + 0) = (this->pulsewidth_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pulsewidth_us >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pulsewidth_us >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pulsewidth_us >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pulsewidth_us);
      *(outbuffer + offset + 0) = (this->max_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_us >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_us >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_us >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_us);
      *(outbuffer + offset + 0) = (this->min_us >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_us >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_us >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_us >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_us);
      *(outbuffer + offset + 0) = (this->speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_channel_name;
      arrToVar(length_channel_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_channel_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_channel_name-1]=0;
      this->channel_name = (char *)(inbuffer + offset-1);
      offset += length_channel_name;
      this->channel_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->channel_id);
      this->safe_us =  ((uint32_t) (*(inbuffer + offset)));
      this->safe_us |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->safe_us |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->safe_us |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->safe_us);
      this->default_us =  ((uint32_t) (*(inbuffer + offset)));
      this->default_us |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->default_us |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->default_us |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->default_us);
      this->pulsewidth_us =  ((uint32_t) (*(inbuffer + offset)));
      this->pulsewidth_us |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pulsewidth_us |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pulsewidth_us |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pulsewidth_us);
      this->max_us =  ((uint32_t) (*(inbuffer + offset)));
      this->max_us |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_us |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->max_us |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->max_us);
      this->min_us =  ((uint32_t) (*(inbuffer + offset)));
      this->min_us |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->min_us |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->min_us |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->min_us);
      this->speed =  ((uint32_t) (*(inbuffer + offset)));
      this->speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->speed);
     return offset;
    }

    virtual const char * getType() override { return "sonicservo/Servo"; };
    virtual const char * getMD5() override { return "649bbc05b43b38164cf0130a4190b6c9"; };

  };

}
#endif
