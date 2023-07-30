//
// Created by linfe on 2023/7/22.
//

#include <fstream>
#include "DataLoader.h"
#include "glog/logging.h"
#include "RecorderType.h"
#include "comm_crc.h"


DataLoader::DataLoader() {

}

void DataLoader::LoadFile(const std::string &filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.good()) {
        LOG(INFO) << "No such file " << filename;
    }
    uint8_t *file_buffer;
    std::ifstream t;
    // open input file
    ifs.seekg(0, std::ios::end);    // go to the end
    file_length = ifs.tellg();           // report location (this is the length)
    ifs.seekg(0, std::ios::beg);    // go back to the beginning
    file_buffer = new uint8_t[file_length];    // allocate memory for a buffer of appropriate dimension
    ifs.read((char *) file_buffer, file_length);       // read the whole file into the buffer
    ifs.close();                    // close file handle
    LOG(INFO) << "file size:" << file_length;
    file_offset = _loadHeader(file_buffer);
    uint8_t *pdata = file_buffer;
    while (file_offset < file_length) {
        pdata = file_buffer + file_offset;
        if (*(uint32_t *) pdata != RECORDER_HEADER) {
            file_offset++;
            continue;
        }
        uint32_t msg_length = *(uint32_t *) (pdata + offsetof(recorder_elements_header, length));
#ifdef ENABLE_CRC_CHECK
        if (crc32_checksum(pdata, msg_length - 4) != 0) {
            LOG(ERROR) << "crc check failed";
            continue;
        }
#endif
        file_offset += msg_length;
        uint32_t msg_id = *(uint32_t *) (pdata + offsetof(recorder_elements_header, msg_id));
        AddData(msg_id, pdata);
    }
}


void DataLoader::AddData(uint32_t msg_id, void *pdata) {
    /*TODO loader目前有bug， 结构体没有对齐的情况下不好搞，不知道数据有没有闲置空间,建议强制对齐*/
    /*TODO 速度可以提一提*/
    auto *msg_header = (recorder_elements_header *) pdata;
    if (data_cfgs.find(msg_id) == data_cfgs.end()) {
        LOG(INFO) << "current id is not configured:" << msg_id;
        return;
    }

    data[msg_id].time.push_back((*(double *) &msg_header->timestamp));
    auto result = structMember2Float(data_cfgs[msg_id].item_config, &msg_header->data);
    for (int i = 0; i < data_cfgs[msg_id].item_config.size(); ++i) {
        data[msg_id].data[i].push_back(result[i]);
    }
}
/**
 * convert byte stream to floats
 * @param items
 * @param pdata
 * @return
 */
std::vector<float> DataLoader::structMember2Float(const std::vector<DataItemConfig> &items, void *pdata) {
    float res = 0;
    auto *addr = (uint8_t *) pdata;
    uint32_t offset = 0;
    std::vector<float> result;
    for (const auto &item: items) {
        auto type = item.type;
        switch (type) {
            case RECORDER_TYPE_uint8_t:
                res = (float) *(uint8_t *) addr;
                offset = 1;
                break;
            case RECORDER_TYPE_int8_t:
                res = (float) *(int8_t *) addr;
                offset = 1;
                break;
            case RECORDER_TYPE_int16_t:
                res = (float) *(int16_t *) addr;
                offset = 2;
                break;
            case RECORDER_TYPE_uint16_t:
                res = (float) *(uint16_t *) addr;
                offset = 2;
                break;
            case RECORDER_TYPE_int32_t:
                res = (float) *(int32_t *) addr;
                offset = 4;
                break;
            case RECORDER_TYPE_uint32_t:
                res = (float) *(uint32_t *) addr;
                offset = 4;
                break;
            case RECORDER_TYPE_int64_t:
                res = (float) *(int64_t *) addr;
                offset = 8;
                break;
            case RECORDER_TYPE_uint64_t:
                res = (float) *(uint64_t *) addr;
                offset = 8;
                break;
            case RECORDER_TYPE_float:
                res = (float) *(float *) addr;
                offset = 4;
                break;
            case RECORDER_TYPE_double:
                res = (float) *(double *) addr;
                offset = 8;
                break;
        }
        result.push_back(res);
        addr += offset;
    }
    return result;
}

int DataLoader::GetProgress() const {
    return int(100 * (file_offset + 1) / file_length);
}

uint32_t DataLoader::_loadHeader(uint8_t *file_buffer) {
    std::map<uint32_t, DataSetConfig> header;
    uint8_t *pbuff = file_buffer;
    if (*(uint32_t *) pbuff != RECORDER_HEADER_START_MARK) {
        return 0;
    }
    pbuff += 4;
    uint32_t dataset_cnt = *(uint32_t *) pbuff;
    pbuff += 4;
    LOG(INFO) << "datasets:" << dataset_cnt;
    for (int i = 0; i < dataset_cnt; i++) {
        DataSetConfig config;
        uint32_t dataset_id = *(uint32_t *) pbuff;
        pbuff += 4;
        config.item_cnt = *(uint32_t *) pbuff;
        pbuff += 4;
        memcpy(config.dataset_name, pbuff, RECORDER_MAX_ITEM_NAME_SIZE);
        pbuff += RECORDER_MAX_ITEM_NAME_SIZE;
        for(int j = 0; j < config.item_cnt; ++j){
            DataItemConfig item;
            item.type = static_cast<RecorderBaseType>(*(uint32_t *) pbuff);
            pbuff += 4;
            memcpy(item.name, pbuff, RECORDER_MAX_ITEM_NAME_SIZE);
            pbuff += RECORDER_MAX_ITEM_NAME_SIZE;
            config.item_config.push_back(item);
        }
        data_cfgs.insert(std::make_pair(dataset_id,config));
    }
    if(*(uint32_t*)pbuff != RECORDER_HEADER_END_MARK){
        LOG(INFO) << "header read error";
        data_cfgs.clear();
        return 0;
    }
    for(auto dataset_config:data_cfgs){
        DataSet dataSet;
        dataSet.data_set_name = dataset_config.second.dataset_name;
        dataSet.sub_data_cnt = dataset_config.second.item_cnt;
        dataSet.data_set = dataset_config.first;
        dataSet.data.reserve(dataSet.sub_data_cnt);
        for (int i = 0; i < dataset_config.second.item_config.size(); ++i) {
            dataSet.data.emplace_back();
            dataSet.data[i].reserve(921600);
            dataSet.subset_name.emplace_back(dataset_config.second.item_config[i].name);
        }
        data.insert(std::make_pair( dataset_config.first,dataSet));
    }

    return pbuff - file_buffer;
}

DataLoader::~DataLoader() = default;
