//
// Created by linfe on 2023/7/22.
//
#include <cstdint>

#ifndef RGIOE_RECORDERTYPE_H
#define RGIOE_RECORDERTYPE_H

#define _1_PACKED __attribute__ ((packed))
#define _4_PACKED __attribute__ ((aligned (4)))

#define MSG_ID(set, id)  ((set << 16u) | (id & 0xffU))
#define RECORDER_HEADER   0XAA55AA55L           // start of elements header
#define RECORDER_END_MARK 0XAA66AA66L           // end of elements

#define RECORDER_MAX_ITEM_NAME_SIZE 16          // Maximum number of characters
#define RECORDER_HEADER_START_MARK  0XBB55BB66L // start of log header
#define RECORDER_HEADER_END_MARK  0XBB77BB88    // end of log header
#define RECORDER_HEADER_MAX_LENGTH (4096)       // Maximum size of log header in bytes

#define RECORDER_MSG_DEF(set, id, name, fmt) \
typedef struct  {                            \
uint32_t header;                              \
uint32_t msg_id ;                             \
uint64_t timestamp;             \
uint32_t length;                \
struct fmt  data;                 \
uint32_t check_sum;             \
uint32_t end_mark;              \
} recorder_msg_##name##_t ;     \
enum{recorder_msg_##name##_id = MSG_ID(set,id)};

#define GET_RECORDER_MSG_ID(name)  recorder_msg_##name##_id

#define CREATE_RECORDER_MSG(name) \
{                                  \
   .header =    RECORDER_HEADER,                 \
   .msg_id = GET_RECORDER_MSG_ID(name),        \
   .length = sizeof(recorder_msg_##name##_t) ,   \
   .end_mark = RECORDER_END_MARK\
}

#define CHECKSUM_RECORDER_CRC32(data)  \
do {                               \
uint32_t crc32 = crc32_checksum((uint8_t*)data,(data)->length - 8);\
(data)->check_sum = ((crc32 & 0xff) << 24u) | ((crc32 & 0xff00) << 8u) | ((crc32 & 0xff0000) >> 8u) | (((crc32 & 0xff000000)>>24));   \
}while(0)


#define PACK_DATA(__D) \
{                      \
   __D                    \
}


#define RECORDER_SET_POSE 0x01

#define RECORDER_SET_SENSOR 0x02

#define RECORDER_SET_DEBUG 0x03

#pragma pack(1)

RECORDER_MSG_DEF(RECORDER_SET_DEBUG, 0x18, kalman,
                  {
                      float matP[15];
                      float acce_bias[3];
                      float acce_scale[3];
                      float gyro_bias[3];
                      float gyro_scale[3];
                      float pos_obs[3];
                  }
)
RECORDER_MSG_DEF(RECORDER_SET_DEBUG, 0x19, state,
                 {
                     float xd[15];
                 }
)
RECORDER_MSG_DEF(RECORDER_SET_DEBUG, 0x20, meas_pos,
                 {
                     float z[3];
                     float r[3];
                 }

)
RECORDER_MSG_DEF(RECORDER_SET_DEBUG, 0x22, align,
                 {
                     float atti[3];
                     float vn[3];
                     float pos[3];
                     float v_norm;
                     uint8_t level_align_finished;
                     uint8_t yaw_align_finished;
                 }
)

RECORDER_MSG_DEF(RECORDER_SET_SENSOR, 0x21, imu,
                  {
                      float gyro[3];
                      float acce[3];
                      float gyro_fix[3];
                      float acce_fix[3];
                  }
)

enum RecorderBaseType {
    RECORDER_TYPE_uint8_t = 0,
    RECORDER_TYPE_int8_t = 1,
    RECORDER_TYPE_uint16_t = 2,
    RECORDER_TYPE_int16_t = 3,
    RECORDER_TYPE_uint32_t = 4,
    RECORDER_TYPE_int32_t = 5,
    RECORDER_TYPE_uint64_t = 6,
    RECORDER_TYPE_int64_t = 7,
    RECORDER_TYPE_float = 8,
    RECORDER_TYPE_double = 9,
};

struct DataItemConfig {
    RecorderBaseType type;
    char name[RECORDER_MAX_ITEM_NAME_SIZE];
};

typedef struct {
    uint32_t header;
    uint32_t msg_id;
    uint64_t timestamp;
    uint32_t length;
    uint32_t data;
    uint32_t check_sum;
    uint32_t end_mark;
} recorder_elements_header;
#pragma pack()
#endif //RGIOE_RECORDERTYPE_H
