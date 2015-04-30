#ifndef ASW_OS_MSG_INT32_H_
#define ASW_OS_MSG_INT32_H_

#include "ros.h"
int serialize_int32(Message* msg, unsigned char *outbuffer)
{
  int offset = 0;
  union {
	int32_t real;
	uint32_t base;
  } u_data;
  u_data.real = *((int32_t*)(msg->data));
  *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
  *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
  *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
  *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
  offset += sizeof(int32_t);
  return offset;
}

int deserialize_int32(Message* msg, unsigned char *inbuffer)
{
  int offset = 0;
  union {
	int32_t real;
	uint32_t base;
  } u_data;
  u_data.base = 0;
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
  *((int32_t*)(msg->data)) = u_data.real;
  offset += sizeof(int32_t);
 return offset;
}



#endif /* ASW_OS_MSG_INT32_H_ */
