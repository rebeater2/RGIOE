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
#include "NavStruct.h"
class IMUSmooth {
 private:
  ImuData imu_ave{};/*均值*/
  ImuData imu_var{};
  ImuData imu_pre{};/*用于计算短时过0*/
  int up_cnt;
  int static_cnt;
  const int width = 32;/*平均数和std窗口*/
  const double static_std_threshold = 1e-3;
  const int static_width = 15;/*连续阈值小于static_std_threshold的判断为静止状态*/
 public:
  //    IMUSmooth(){};
  IMUSmooth();

  void Update(const ImuData &imu);

  ImuData getSmoothedIMU();

  double getStd();

  bool isStatic();
};

#endif //LOOSELYCOUPLE2020_CPP_SRC_STATICDETECT_H_