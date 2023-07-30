//
// Created by linfe on 2023/7/22.
//

#ifndef STATICGRAPH_DATALOADER_H
#define STATICGRAPH_DATALOADER_H

#include <string>
#include <map>
#include <vector>
#include "Recorder.h"
#define ENABLE_CRC_CHECK



struct DataItemCfg {
    RecorderBaseType type;
    char name[25];
};

struct RecorderDataSetCfg {
    std::string name;
    uint32_t msg_id;
    std::vector<DataItemCfg> items;
};


class DataSet{
public:
    uint32_t data_set;
    uint32_t sub_data_cnt;
    std::string data_set_name;
    std::vector<std::string> subset_name;
    std::vector<std::vector<float>> data;
    std::vector<double> time;
};


class DataLoader {
public:
    DataLoader();
    ~DataLoader();

public:
    void LoadFile(const std::string &filename);
    uint32_t _loadHeader(uint8_t *file_buffer);
    void AddData(uint32_t msg_id,void *pdata);

    int GetProgress()const;

public:
    std::map<uint32_t,DataSet> data;
    std::map<uint32_t ,DataSetConfig> data_cfgs;
private:
    uint64_t file_length,file_offset = 0;
private:
    static std::vector<float> structMember2Float(const std::vector<DataItemConfig>& types, void *pdata);
};


#endif //STATICGRAPH_DATALOADER_H
