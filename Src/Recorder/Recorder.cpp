
#include "Recorder.h"
#include <map>
#include "RecorderHeaderDef.h"

#define RECORDER_ADD_DATASET(target,id,name,config) \
 do{ DataItemConfig items[] = config; \
DataSetConfig dataset_cfg = {\
        .dataset_name = name,\
        .item_cnt = sizeof(items) / sizeof(items[0]),\
};\
for (auto &item: items) {\
dataset_cfg.item_config.push_back(item);\
}\
target.insert(std::make_pair(id, dataset_cfg));     \
} while(0) \


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
    HEADERCONFIG(header_config);

}


/**
 * Write self-parse header to parse the file without previous configure
 */
void Recorder::WriteHeader() {
    if (!ofs.good()) {
        return;
    }
    /*write header*/
    uint8_t *pheader = header_buffer;
    *(uint32_t *) pheader = RECORDER_HEADER_START_MARK;
    pheader += 4;
    *(uint32_t *) pheader = header_config.size();
    pheader += 4;
    for (auto dataset_config: header_config) {
        *(uint32_t *) pheader = dataset_config.first;
        pheader += 4;
        *(uint32_t *) pheader = dataset_config.second.item_config.size();
        pheader += 4;
        memcpy(pheader, dataset_config.second.dataset_name, RECORDER_MAX_ITEM_NAME_SIZE);
        pheader += RECORDER_MAX_ITEM_NAME_SIZE;
        for (auto &item: dataset_config.second.item_config) {
            *(uint32_t *) pheader = item.type;
            pheader += 4;
            memcpy(pheader, item.name, RECORDER_MAX_ITEM_NAME_SIZE);
            pheader += RECORDER_MAX_ITEM_NAME_SIZE;
        }
    }
    *(uint32_t *) pheader = RECORDER_HEADER_END_MARK;
    pheader += 4;
    ofs.write((const char *) header_buffer, pheader - header_buffer);
    std::cout << "header size:" << (float) (pheader - header_buffer) / 1024.0f << "kb";
    ofs.flush();
}
