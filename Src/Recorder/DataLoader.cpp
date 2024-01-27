//
// Created by linfe on 2023/7/22.
//
#include "comm_crc.h"
#include "DataLoader.h"

#include "glog/logging.h"

#include <fstream>
#include <utility>


DataLoader::DataLoader() :
        recorder_version{0},
        file_length{0},
        file_offset{0} {

}

void DataLoader::LoadFile(const std::string &filename) {
    error_strings.clear();
    errorCode = DATALOADER_OK;
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.good()) {
        error_strings.emplace_back( "No such file " + filename);
        return;
    }
    uint8_t *file_buffer;
    std::ifstream t;
    // open input file
    ifs.seekg(0, std::ios::end);    // go to the end
    file_length = ifs.tellg();           // report location (this is the length)
    ifs.seekg(0, std::ios::beg);    // go back to the beginning
    file_buffer = new uint8_t[file_length];    // allocate memory for a buffer of appropriate dimension
    ifs.read((char *) file_buffer, (long long) file_length);       // read the whole file into the buffer
    ifs.close();                    // close file handle
    file_offset = _loadHeader(file_buffer);
    header_size = file_offset;
    if (!file_offset) {
        errorCode = DATALOADER_NO_HEADER;
        return;
    }
    uint8_t *pdata;
    while (file_offset < file_length) {
        pdata = file_buffer + file_offset;
        if (*(uint32_t *) pdata != RECORDER_HEADER) {
            file_offset++;
            continue;
        }
        uint32_t msg_length = *(uint32_t *) (pdata + offsetof(recorder_elements_header, length));
#ifdef ENABLE_CRC_CHECK
        if (crc32_checksum(pdata, msg_length - 4) != 0) {
            error_strings.emplace_back("CRC check failed");
            continue;
        }
#endif
        file_offset += msg_length;
        uint32_t msg_id = *(uint32_t *) (pdata + offsetof(recorder_elements_header, msg_id));
        AddData(msg_id, pdata);
    }
}


void DataLoader::AddData(uint32_t msg_id, void *pdata) {
    auto *msg_header = (recorder_elements_header *) pdata;
    if (data_cfgs.find(msg_id) == data_cfgs.end()) {
        error_strings.emplace_back("current id is not configured:" + std::to_string(msg_id));
        return;
    }
    if (data.find(msg_id) == data.end()) {
        DataSet dataSet;
        dataSet.data_set_name = data_cfgs[msg_id].dataset_name;
        dataSet.sub_data_cnt = data_cfgs[msg_id].item_cnt;
        dataSet.data_set = msg_id;
        dataSet.data.reserve(dataSet.sub_data_cnt);
        for (int i = 0; i < data_cfgs[msg_id].item_config.size(); ++i) {
            dataSet.data.emplace_back();
            dataSet.data[i].reserve(921600);
            dataSet.subset_name.emplace_back(data_cfgs[msg_id].item_config[i].name);
        }
        data.insert(std::make_pair(msg_id, dataSet));
    }
    data[msg_id].time.push_back((*(double *) &msg_header->timestamp));
    std::vector<float> result;
    auto offset = structMember2Float(data_cfgs[msg_id].item_config, &msg_header->data,result);
    if (msg_header->length != sizeof(recorder_elements_header) + offset - 4){
        errorCode = DATALOADER_HEADER_DATA_NOT_MATCH;
        errorcodes[msg_id] = DATALOADER_HEADER_DATA_NOT_MATCH;
    }
    for (int i = 0; i < data_cfgs[msg_id].item_config.size(); ++i) {
        data[msg_id].data[i].push_back(result[i]);
    }
}


 uint32_t DataLoader::structMember2Float(const std::vector<DataItemConfig> &items, void *pdata,std::vector<float> &result) {
    float res = 0;
    auto *start = (uint8_t *) pdata;
    auto *addr = (uint8_t *) pdata;
    uint32_t offset = 0;
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
    return (addr - start);
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
    uint32_t header_length = *(uint32_t *) pbuff;
    pbuff += 4;
    if (crc32_checksum(file_buffer, header_length - 4) != 0) {
        error_strings.emplace_back("file header was damaged");
        errorCode = DATALOADER_HEADER_CHECKFAILED;
        return 0;
    }
    header_size = header_length;
    recorder_version = *(float *) pbuff;
    pbuff += 4;
    uint32_t dataset_cnt = *(uint32_t *) pbuff;
    pbuff += 4;
    for (int i = 0; i < dataset_cnt; i++) {
        DataSetConfig config;
        uint32_t dataset_id = *(uint32_t *) pbuff;
        pbuff += 4;
        config.item_cnt = *(uint32_t *) pbuff;
        pbuff += 4;
        memcpy(config.dataset_name, pbuff, RECORDER_MAX_ITEM_NAME_SIZE);
        pbuff += RECORDER_MAX_ITEM_NAME_SIZE;
        for (int j = 0; j < config.item_cnt; ++j) {
            DataItemConfig item{};
            item.type = static_cast<RecorderBaseType>(*(uint32_t *) pbuff);
            pbuff += 4;
            memcpy(item.name, pbuff, RECORDER_MAX_ITEM_NAME_SIZE);
            pbuff += RECORDER_MAX_ITEM_NAME_SIZE;
            config.item_config.push_back(item);
        }
        data_cfgs.insert(std::make_pair(dataset_id, config));
        errorcodes.insert(std::make_pair(dataset_id,DATALOADER_OK));
    }
    pbuff += 4; // skip crc32
    if (*(uint32_t *) pbuff != RECORDER_HEADER_END_MARK) {
        error_strings.emplace_back("Header endmark not found");
        errorCode = DATALOADER_HEADER_ENDMARK_NOT_FOUND;
        data_cfgs.clear();
        return 0;
    }
/*    for (auto dataset_config: data_cfgs) {
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
        data.insert(std::make_pair(dataset_config.first, dataSet));
    }*/

    return pbuff - file_buffer;
}

std::string DataLoader::GetSummaryString() const {
    char buff[4096];
    int offset = 0;
/*    offset += sprintf(buff + offset,"\nHeader(size %llu) info:\n",header_size);
    for(const auto & cfg:data_cfgs){
        offset += sprintf(buff + offset,"%-16s(0X%X)\n",cfg.second.dataset_name,cfg.first);
        for(auto item:cfg.second.item_config){
            offset += sprintf(buff + offset,"|___%-16s\t %x\n",item.name,item.type);
        }
    }//*/
    offset += sprintf(buff + offset, "\nHeader size %lu\tRecorder version:%.2f \n", header_size, recorder_version);
    offset += sprintf(buff + offset, "Valid dataset(%.2zu) incuding:\n", data.size());
    offset += sprintf(buff + offset, "\t%16s %10s %8s %8s\n", "dataset", "ID", "size", "subsets");
    for (const auto &d: data) {
        offset += sprintf(buff + offset, "\t%16s %10x %8zu %8d\n", d.second.data_set_name.c_str(), d.first,
                          d.second.time.size(), d.second.sub_data_cnt);
    }
    if (!error_strings.empty()) {
        offset += sprintf(buff + offset, "Errors:\n");
        for (auto &error: error_strings) {
            offset += sprintf(buff + offset, "%s\n", error.c_str());
        }
    }
    return std::string{buff};
}

const DataLoaderErrorCode &DataLoader::GetErrorCode() const {
    return errorCode;
}

const std::list<std::string> &DataLoader::GetErrorString() const {
    return error_strings;
}

DataLoaderErrorCode &DataLoader::GetErrorCodeByID(DataLoader::msg_id_t id) {
    return errorcodes[id];
}


DataLoader::~DataLoader() = default;

DataLoaderCsvWriter::DataLoaderCsvWriter(std::string filename) : filename(std::move(filename)) {

}

void DataLoaderCsvWriter::Save2Csv(const DataSet &dataSet) {
    std::ofstream ofs(filename);
    uint32_t length = dataSet.time.size();
    uint32_t subdata_cnt = dataSet.data.size();
    ofs << "time" << ",";
    for (auto &name: dataSet.subset_name) {
        ofs << name << ',';
    }
    ofs << "\n";
    for (uint32_t i = 0; i < length; ++i) {
        ofs << dataSet.time[i] << ",";
        for (int j = 0; j < subdata_cnt; j++) {
            ofs << dataSet.data[j][i] << ",";
        }
        ofs << '\n';
    }
    ofs.flush();
    ofs.close();
}
#ifdef RECORDER_ENABLE_EXPORT_MATLAB
DataLoaderMatWriter::DataLoaderMatWriter(std::string filename) : filename(std::move(filename)) {
}
#include "mat.h"
DataLoaderMatWriter::MatErrorCode DataLoaderMatWriter::Save2Mat(const std::map<uint32_t,DataSet> &dataSets) {
    MATFile *pmat = matOpen(filename.c_str(), "w");
    if (pmat == nullptr) {
        err_msg = "Mat file create failed:" + filename;
        return MatExportFileCreateFailed;
    }
    for (const auto &keyvalue:dataSets){
       auto dataSet = keyvalue.second;
        if (dataSet.sub_data_cnt != dataSet.subset_name.size()) {
            err_msg = "Dataset Error";
            return MatExportFileDataSetError;
        }
        mwSize dims[2];
        dims[0] = dataSet.sub_data_cnt + 1;
        dims[1] = dataSet.time.size();
        char **field_names = new char *[dataSet.sub_data_cnt + 1];
        for (int i = 0; i < dataSet.sub_data_cnt + 1; i++) {
            field_names[i] = new char[25];
        }
        strcpy(field_names[0], "time");
        for (int i = 0; i < dataSet.sub_data_cnt; ++i) {
            strcpy(field_names[i + 1], dataSet.subset_name[i].c_str());
        }
        mxArray *array = mxCreateCharMatrixFromStrings(dataSet.sub_data_cnt, (const char **) (field_names));
        if (array == nullptr) {
            printf("array closing file\n");
        }
        char subset_nams[32];
        sprintf(subset_nams,"%s_field_names",dataSet.data_set_name.c_str());
        matPutVariable(pmat, subset_nams, array);
        mxArray *field_value;

        field_value = mxCreateDoubleMatrix(dims[1], dims[0], mxREAL);
        for (int i = 0; i < dataSet.time.size(); i++) {
//        memcpy((void *) (mxGetPr(field_value)), (void *) data, sizeof(data));
            *(mxGetDoubles(field_value) + i) = dataSet.time[i];
        }
        for (int j = 0; j < dataSet.sub_data_cnt; ++j) {
            for (int i = 0; i < dataSet.time.size(); i++) {
#if MX_HAS_INTERLEAVED_COMPLEX
                *(mxGetDoubles(field_value) + (j+1) * dataSet.time.size() + i) = dataSet.data[j][i];
#else
                *(mxGetPr(field_value)+ (j+1) * dataSet.time.size() + i) = dataSet.data[j][i];
#endif
                /* Use mxSetFieldByNumber instead of mxSetField for efficiency
                 * mxSetField(array,i,"name",mxCreateString(friends[i].name); */
//            mxSetFieldByNumber(array, i, j+1, field_value);
            }
        }
        matPutVariable(pmat, dataSet.data_set_name.c_str(), field_value);
    }
    if (matClose(pmat) != 0) {
        printf("Error closing file\n");
    }
    return MatExportOK;
}

std::string DataLoaderMatWriter::GetErrorMsg() const {
    return err_msg;
}
#endif