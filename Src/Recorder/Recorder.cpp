#include "Recorder.h"
#include "comm_crc.h"
#include <map>

#define RECORDER_ADD_DATASET(name, config) \
do{DataItemConfig items[] = config; \
DataSetConfig dataset_cfg = {\
         #name,\
         sizeof(items) / sizeof(items[0]), \
         {}\
};\
for (auto &item: items) {\
dataset_cfg.item_config.push_back(item);\
}\
header_config.insert(std::make_pair(GET_RECORDER_MSG_ID(name), dataset_cfg));} while(0)


Recorder::~Recorder() {
    ofs.flush();
    ofs.close();
    delete header_buffer;
}


void Recorder::OpenFile(const char *filename) {
    ofs.open(filename, std::ios::binary);
}

Recorder::Recorder() {
    header_buffer = new uint8_t[RECORDER_HEADER_MAX_LENGTH];
    /*create header config*/

#define RECORDER_TYPE_DEF_START(name,set,id) \
    do{ DataItemConfig items[] = {

#define RECORDER_ITEM_DEF(type,name) {RECORDER_TYPE_##type,#name},


#define RECORDER_TYPE_DEF_END(name) \
     };         \
    DataSetConfig config = {\
            .dataset_name = #name,\
            .item_cnt = sizeof(items)/sizeof(items[0]),\
            .item_config = {},\
    };\
    for(auto item:items){\
        config.item_config.push_back(item);\
    }\
    header_config.insert(std::make_pair(recorder_msg_##name##_id, config));}while(0);

#include "RecorderDef.h"


#ifdef RECORDER_HEADERCONFIG
    RECORDER_HEADERCONFIG
#endif
}


/**
 * Write self-parse header to parse the file without previous configure
 */
void Recorder::WriteHeader() {
    if (!ofs.good()) {
        return;
    }
    if (header_config.empty()){
        return;
    }
    /* write header */
    uint8_t *pheader = header_buffer;
    /* start mark */
    *(uint32_t *) pheader = RECORDER_HEADER_START_MARK;
    pheader += 4;

    /* length of header(unknown yet) */
    uint8_t *header_length_addr = pheader;
    pheader += 4;
    /* version of recorder */
    *(float *) pheader = RECORDER_MAJOR_VERSION + RECORDER_MINOR_VERSION / 100.0;
    pheader += 4;
    /* number of datasets */
    *(uint32_t *) pheader = header_config.size();
    pheader += 4;
    /* configure for each dataset */
    for (auto dataset_config: header_config) {
        /* dataset ID */
        *(uint32_t *) pheader = dataset_config.first;
        pheader += 4;
        /* number of members for dataset */
        *(uint32_t *) pheader = dataset_config.second.item_config.size();
        pheader += 4;
        /* dataset name */
        memcpy(pheader, dataset_config.second.dataset_name, RECORDER_MAX_ITEM_NAME_SIZE);
        pheader += RECORDER_MAX_ITEM_NAME_SIZE;
        /* configure for each item of dataset */
        for (auto &item: dataset_config.second.item_config) {
            /* item type */
            *(uint32_t *) pheader = item.type;
            pheader += 4;
            /* item name */
            memcpy(pheader, item.name, RECORDER_MAX_ITEM_NAME_SIZE);
            pheader += RECORDER_MAX_ITEM_NAME_SIZE;
        }
    }
    /* calc header length */
    *(uint32_t*)header_length_addr = pheader - header_buffer + 8;
    /* calc crc32 checksum */
    uint32_t crc = crc32_checksum(header_buffer,pheader - header_buffer);
    *(uint32_t*)pheader = REVERT_CRC32(crc);
    pheader += 4;
    /* end of header */
    *(uint32_t *) pheader = RECORDER_HEADER_END_MARK;
    pheader += 4;
    ofs.write((const char *) header_buffer, pheader - header_buffer);
    ofs.flush();
}

Recorder &Recorder::GetInstance() {
    static Recorder recorder;
    return recorder;
}

void Recorder::Initialize(const char *argv0) {
    int offset = 0;
    if (!argv0) {
        offset += sprintf(rcd_filename, "recorder");
    } else {
        offset += sprintf(rcd_filename, "%s", argv0);
    }
    time_t t = time(nullptr);
    struct tm *stime = localtime(&t);
    sprintf(rcd_filename + offset, ".%04d%02d%02d_%02d%02d%02d.rcd",
            stime->tm_year + 1900,
            stime->tm_mon,
            stime->tm_mday,
            stime->tm_hour,
            stime->tm_min,
            stime->tm_sec
    );
    OpenFile(rcd_filename);
    WriteHeader();
}

char *Recorder::GetRcdFilename() {
    return rcd_filename;
}
