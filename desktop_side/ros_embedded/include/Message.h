typedef struct Message
{
   void* data;
   unsigned short length;
} Message;

typedef enum {RCL_MSG_TYPE_FLOAT, RCL_MSG_TYPE_UINT32} MessageType;

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
  *((int*)(msg->data)) = u_data.real;
  offset += sizeof(int32_t);
 return offset;
}

int deserialize_float32(Message* msg, unsigned char *inbuffer)
{
  int offset = 0;
  union {
   float real;
   uint32_t base;
  } u_data;
  u_data.base = 0;
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
  u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
  *((float*)(msg->data)) = u_data.real;
  offset += sizeof(float);
 return offset;
}

int serialize_int32(Message* msg, unsigned char *outbuffer)
{
  int offset = 0;
  union {
   int32_t real;
   uint32_t base;
  } u_data;
  u_data.real = *((int*)(msg->data));
  *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
  *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
  *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
  *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
  offset += sizeof(int32_t);
  return offset;
}

int serialize_float32(Message* msg, unsigned char *outbuffer)
{
  int offset = 0;
  union {
   float real;
   uint32_t base;
  } u_data;
  u_data.real = *((float*)(msg->data));
  *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
  *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
  *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
  *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
  offset += sizeof(float);
  return offset;
}