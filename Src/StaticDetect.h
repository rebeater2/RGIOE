/**
* @file StaticDetect.h in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 8/21/21 10:16 PM
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_SRC_STATICDETECT_H_
#define LOOSELYCOUPLE2020_CPP_SRC_STATICDETECT_H_
/* IMU 平均数*/
#include "RgioeDataType.h"

class IMUSmooth {
private:
    RgioeImuData imu_ave{};/*均值*/
    RgioeImuData imu_var{};
    RgioeImuData imu_pre{};/*用于计算短时过0*/
    int up_cnt;
    int static_cnt;
    int width = 32;/*平均数和std窗口*/
    RgioeFloatType static_std_threshold = 1e-3;
    int static_width = 15;/*连续阈值小于static_std_threshold的判断为静止状态*/
    bool is_static_;
public:
    //    IMUSmooth(){};
    IMUSmooth();

    IMUSmooth(float threshold, int static_width, int window);

    void Update(const RgioeImuData &imu);

    RgioeImuData getSmoothedIMU();

    RgioeFloatType getStd();

    bool isStatic() const;
};

#endif //LOOSELYCOUPLE2020_CPP_SRC_STATICDETECT_H_
