//
// Created by linfe on 12/21/2023.
//

#ifndef RGIOE_DATAMANAGER_H
#define RGIOE_DATAMANAGER_H


#include "rgioe.h"
#include "FileIO.h"
#include "Config.h"
#include <memory>
#include <vector>
#include <list>
#include <algorithm>
#include <climits>
/*template <typename ForwardIterator>
inline size_t argmin(const ForwardIterator begin, const ForwardIterator end)
{
    return std::distance(begin, std::min_element(begin, end));
}

template <typename ForwardIterator>
inline size_t argmax(const ForwardIterator begin, const ForwardIterator end)
{
    return std::distance(begin, std::max_element(begin, end));
}*/
/**
 * implementation of argmin
 * @tparam T
 * @param array
 * @param num
 * @return
 */
template<typename T>
inline int argmin(T array[], int num){
    T min_value = INT_MAX;
    int arg_min = 0;
    for(int i = 0; i < num; ++i){
        if (array[i] < min_value){
            min_value = array[i];
            arg_min = i;
        }
    }
    return arg_min;
}




typedef enum{
    IMU_FILE_ATTR_BIN = 0b00000001,
    IMU_FILE_ATTR_ASCII = 0b00000000,
    IMU_FILE_ATTR_FRD  = 0b000000010,
    IMU_FILE_ATTR_RFU  = 0b000000010,
}ImuFileAttr_t;

class DataSeqCheck{
public:
    DataSeqCheck() = default;
    ~DataSeqCheck() = default;
    void Update(const std::shared_ptr<BaseData_t> data);

private:
    uint32_t data_cnt = 0;
    TimeStamp_t last_time = 0;
    TimeStamp_t delta_time = 0;
    TimeStamp_t average_delta_time = 0;
    TimeStamp_t max_delta_time = 0;
    TimeStamp_t gnss_delay_time = 0;
};


class DataManager{
public:
    DataManager();
    ~DataManager();
    DataManager &AddFile(const GnssConfig& config);
    DataManager &AddFile(const IMUConfig& config);
    std::shared_ptr<BaseData_t> GetNextData();
    void MoveToTime(TimeStamp_t target);
    void Reset();
private:
    DataSeqCheck checker;
    using data_queue_t = std::vector<std::shared_ptr<BaseData_t>>;
    static const int max_que_num = 3;               /*! 最大数据队列          */

    /**
     * 添加新的数据列队，更新这里的索引，放到对应的queue里即可，最大ID < max_que_num
     */
    static  const int imu_que_id = 0;               /*! imu 队列索引          */
    static  const int gnss_que_id = 1;              /*! GNSS队列索引          */
    static  const int odo_que_id = 2;               /*! 里程计队列索引         */

    data_queue_t que[max_que_num];                  /*! 存放数据队列           */
    int position[max_que_num];                      /*! 当前队列访问到的位置    */
    TimeStamp_t t[max_que_num]{};                   /*! 临时变量，时间头部时间  */
    int que_num = 0;                                /*! 当前读入队列数         */
};


#endif //RGIOE_DATAMANAGER_H
