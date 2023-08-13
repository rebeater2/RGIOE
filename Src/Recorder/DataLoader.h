//
// Created by linfe on 2023/7/22.
//

#ifndef STATICGRAPH_DATALOADER_H
#define STATICGRAPH_DATALOADER_H

#include "Recorder.h"

#include <string>
#include <map>
#include <vector>
#include <list>

/* disable CRC check will load faster*/
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

enum DATALOADER_ERROR_CODE{
    DATALOADER_OK = 0,
    DATALOADER_NO_HEADER = 1,
    DATALOADER_HEADER_CHECKFAILED = 2,
    DATALOADER_HEADER_ENDMARK_NOT_FOUND = 2,
};

class DataLoader {
public:
    DataLoader();
    ~DataLoader();

public:
    void LoadFile(const std::string &filename);
    [[nodiscard]] int GetProgress()const;
    [[nodiscard]] std::string GetSummaryString() const;
    [[nodiscard]] DATALOADER_ERROR_CODE GetErrorCode() const;
private:
    uint32_t _loadHeader(uint8_t *file_buffer);
    void AddData(uint32_t msg_id,void *pdata);
public:
    std::map<uint32_t,DataSet> data;
    std::map<uint32_t ,DataSetConfig> data_cfgs;
    float recorder_version;
private:
    uint64_t file_length,file_offset,header_size;
    DATALOADER_ERROR_CODE errorCode;
    std::list<std::string> error_strings;

private:
    static std::vector<float> structMember2Float(const std::vector<DataItemConfig>& types, void *pdata);
};


#endif //STATICGRAPH_DATALOADER_H
