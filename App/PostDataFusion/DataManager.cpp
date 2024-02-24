//
// Created by linfe on 12/21/2023.
//

#include "DataManager.h"
#include "FileIO.h"

#if ENABLE_FUSION_RECORDER
#include <Recorder/RecorderType.h>
#endif

#include <glog/logging.h>

std::shared_ptr<BaseData_t> DataManager::GetNextData() {
    std::shared_ptr<BaseData_t> data = nullptr;
    for (int i = 0; i < que_num; ++i) {
        if (position[i] < que[i].size()) {
            t[i] = que[i][position[i]]->time;
        } else {
            return nullptr;
        }
    }
    int select_que = argmin(t, que_num);
    data = que[select_que][position[select_que]++];
    checker.Update(data);
    return data;
}

DataManager &DataManager::AddFile(const GnssConfig &config) {
    if(!config.enable) return *this;
    LOG(INFO) << "add GNSS file:" << config.file_path;
    GnssReader reader{config.file_path, config.format};
    while (reader.IsOk()) {
        auto gnss = reader.ReadNext();
        if (gnss == nullptr) break;
        que[gnss_que_id].push_back(gnss);
    }
    que_num++;
    LOG(INFO) << "GNSS size:" << que[gnss_que_id].size();
    return *this;
}

DataManager &DataManager::AddFile(const IMUConfig &config) {
    LOG(INFO) << "add IMU file:" << config.file_path;
    IMUReader reader{config.file_path, config.format, config.frame, false, config.d_rate};
    while (reader.IsOk()) {
        auto data = reader.ReadNext();
        if (data == nullptr) break;
        que[imu_que_id].push_back(data);
    }
    que_num++;
    LOG(INFO) << "IMU queue size:" << que[imu_que_id].size();
    return *this;
}

DataManager::DataManager() {
    for (auto &p: position) {
        p = 0;
    }
    for (auto &p: t) {
        p = 0;
    }
}

DataManager::~DataManager() {

}

void DataManager::MoveToTime(TimeStamp_t target) {
    std::shared_ptr<BaseData_t> data;
    do{
        data = GetNextData();
        if (data == nullptr) break;
    } while(data->time < target);
}

void DataManager::Reset() {
    for(auto &d: position){
        d = 0;
    }
}


RgioeGnssData GnssData_t::toRgioeData() const {
    return {.lat=lat,
            .lon=lon,
            .height=height,
            .pos_std= {pos_std[0], pos_std[1], pos_std[2]},
            .gpst = time,
            .mode = SPP,

    };
}

GnssData_t::GnssData_t() {
    type = DATA_TYPE_GNSS;
}

RgioeImuData ImuData_t::toRgioeData() const {
    return {time, gyro[0], gyro[1], gyro[2], acce[0], acce[1], acce[2]};
}

ImuData_t::ImuData_t() {
    type = DATA_TYPE_IMU;
}

void DataSeqCheck::Update(const std::shared_ptr<BaseData_t> data) {
    if (last_time > 0)
        delta_time = data->time - last_time;
    else
        delta_time = 0;
    data_cnt++;
    average_delta_time = (average_delta_time * (data_cnt - 1) + delta_time)/data_cnt;
    if (delta_time > max_delta_time){
        max_delta_time = delta_time;
    }
    if (data->type == DataType_t::DATA_TYPE_GNSS){
        auto gnss = std::dynamic_pointer_cast<GnssData_t>(data);
        gnss_delay_time = gnss->time - gnss->gpst;
    }
    last_time = data->time;
#if ENABLE_FUSION_RECORDER
    recorder_msg_time_check_t rcd = CREATE_RECORDER_MSG(time_check);
    rcd.timestamp = data->time;
    rcd.data.delta_time = delta_time;
    rcd.data.average_delta_time = average_delta_time;
    rcd.data.max_delta_time = max_delta_time;
    rcd.data.gnss_delay_time = gnss_delay_time;
    Recorder::GetInstance().Record(&rcd);
#endif
}
