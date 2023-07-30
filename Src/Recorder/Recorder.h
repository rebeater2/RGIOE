//
// Created by linfe on 2023/7/22.
//

#ifndef RGIOE_RECORDER_H
#define RGIOE_RECORDER_H
#include "RecorderType.h"
#include "comm_crc.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <map>


struct DataSetConfig {
    char dataset_name[RECORDER_MAX_ITEM_NAME_SIZE];
    uint32_t item_cnt;
    std::vector<DataItemConfig>  item_config;
};



class Recorder {
private:
    std::ofstream ofs;
    uint8_t *header_buffer;
    std::map<uint32_t, DataSetConfig> header_config;
public:
    void WriteHeader();
public:
    Recorder();
    void OpenFile(const char *filename);
    ~Recorder();
    template<typename T>
    void Record(void *data);
};


template<typename T>
void Recorder::Record(void *data) {
    ofs.write((const char *)data,sizeof(T));
}

#endif //RGIOE_RECORDER_H
