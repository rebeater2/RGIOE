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

enum DataLoaderErrorCode{
    DATALOADER_OK = 0,                                  /*! 正常解析 */
    DATALOADER_NO_HEADER = 1,                           /*! 没找到子解析头 */
    DATALOADER_HEADER_CHECKFAILED = 2,                  /*! 子解析头校验失败 */
    DATALOADER_HEADER_ENDMARK_NOT_FOUND = 3,            /*! 没有找到子解析尾 */
    DATALOADER_HEADER_DATA_NOT_MATCH = 4                /*! 子解析配置和数据文件不匹配 */
};

class DataLoader {
public:
    DataLoader();
    ~DataLoader();
    using msg_id_t = uint32_t;

public:
    void LoadFile(const std::string &filename);
    [[nodiscard]] int GetProgress()const;
    [[nodiscard]] std::string GetSummaryString() const;
    [[nodiscard]] const DataLoaderErrorCode &GetErrorCode() const;
    [[nodiscard]] DataLoaderErrorCode &GetErrorCodeByID(msg_id_t id) ;
    [[nodiscard]] const std::list<std::string> &GetErrorString() const;
private:
    uint32_t _loadHeader(uint8_t *file_buffer);
    void AddData(msg_id_t msg_id,void *pdata);
public:
    std::map<msg_id_t,DataSet> data;
    std::map<msg_id_t ,DataSetConfig> data_cfgs;
    float recorder_version;
private:
    uint64_t file_length,file_offset,header_size;
    DataLoaderErrorCode errorCode;
    std::map<msg_id_t,DataLoaderErrorCode> errorcodes;
    std::list<std::string> error_strings;

private:

    /**
     * convert byte stream to floats
     * @param types
     * @param pdata
     * @param result
     * @return
     */
    static uint32_t structMember2Float(const std::vector<DataItemConfig>& types, void *pdata, std::vector<float>& result);
};
class DataLoaderCsvWriter{
public:
    explicit DataLoaderCsvWriter(std::string filename);
    void Save2Csv(const DataSet &dataSet);
private:
    std::string  filename;
};
#ifdef RECORDER_ENABLE_EXPORT_MATLAB
class DataLoaderMatWriter{
public:
    enum MatErrorCode{
        MatExportOK = 0,
        MatExportFileCreateFailed = 1,
        MatExportFileDataSetError = 1,
    };
public:
    explicit DataLoaderMatWriter(std::string filename);
    MatErrorCode Save2Mat(const std::map<uint32_t,DataSet> &dataSets);
    [[nodiscard]] std::string GetErrorMsg()const;
private:
    std::string  filename;
    std::string err_msg;
};
#endif // RECORDER_ENABLE_EXPORT_MATLAB
#endif //STATICGRAPH_DATALOADER_H
