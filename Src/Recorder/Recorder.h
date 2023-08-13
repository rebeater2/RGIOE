//
// Created by linfe on 2023/7/22.
//

#ifndef RGIOE_RECORDER_H
#define RGIOE_RECORDER_H
#include "comm_crc.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cstring>

#define RECORDER_HEADER                     0XAA55AA55L     // start of elements header
#define RECORDER_END_MARK                   0XAA66AA66L     // end of elements

#define RECORDER_MAX_ITEM_NAME_SIZE         16              // Maximum number of characters
#define RECORDER_HEADER_START_MARK          0XBB55BB66L     // start of log header
#define RECORDER_HEADER_END_MARK            0XBB77BB88      // end of log header
#define RECORDER_HEADER_MAX_LENGTH          (4096)          // Maximum size of log header in bytes

#define RECORDER_MAJOR_VERSION              1               // Major version of recorder(start from 1)
#define RECORDER_MINOR_VERSION              1               // Minor version of recorder(1~100)

#define MSG_ID(set, id)                     ((set << 16u) | (id & 0xffU))
#define RECORDER_TIMESTAMP_TYPE             double

#define RECORDER_MSG_DEF(set, id, name, fmt) \
typedef struct  {                            \
uint32_t header;                              \
uint32_t msg_id ;                             \
RECORDER_TIMESTAMP_TYPE timestamp;             \
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

typedef struct {
    uint32_t header;
    uint32_t msg_id;
    RECORDER_TIMESTAMP_TYPE timestamp;
    uint32_t length;
    uint32_t data;
    uint32_t check_sum;
    uint32_t end_mark;
} recorder_elements_header;

struct DataItemConfig {
    RecorderBaseType type;
    char name[RECORDER_MAX_ITEM_NAME_SIZE];
};

struct DataSetConfig {
    char dataset_name[RECORDER_MAX_ITEM_NAME_SIZE];
    uint32_t item_cnt;
    std::vector<DataItemConfig> item_config;
};


class Recorder {
public:
    ~Recorder();
    void Initialize(const char *argv0 = nullptr);
    static Recorder &GetInstance();
    template<typename T>
    void Record(T *data);
    char *GetRcdFilename();

private:
    std::ofstream ofs;
    char rcd_filename[128];
    uint8_t *header_buffer;
    std::map<uint32_t, DataSetConfig> header_config;
private:
    void WriteHeader();
    void OpenFile(const char *filename);
private:
    Recorder();
};


template<typename T>
void Recorder::Record(T *data) {
    ofs.write((const char *) data, sizeof(T));
}

#endif //RGIOE_RECORDER_H
