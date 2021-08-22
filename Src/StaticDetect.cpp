/**
* @file StaticDetect.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 8/21/21 10:16 PM
* @version 1.0
**/


#include "StaticDetect.h"
#include <cmath>
IMUSmooth::IMUSmooth() {
  imu_ave = {0, 0, 0, 0, 0, 0, 0};
  imu_var = {0, 0, 0, 0, 0, 0, 0};

  up_cnt = 0;
  static_cnt = 0;
}

/**
 * 滑动计算IMU的均值和方差
 * @param imu
 */
void IMUSmooth::Update(const ImuData &imu) {
  up_cnt++;
  if (up_cnt < width) {/*小于窗口长度*/
    for (int i = 0; i < 3; i++) {
      imu_ave.acce[i] = (imu_ave.acce[i] * (up_cnt - 1) + imu.acce[i]) / up_cnt;
      imu_ave.gyro[i] = (imu_ave.gyro[i] * (up_cnt - 1) + imu.gyro[i]) / up_cnt;

      imu_var.acce[i] = (imu_var.acce[i] * (up_cnt - 1) +
      	(imu.acce[i] - imu_ave.acce[i]) * (imu.acce[i] - imu_ave.acce[i])) / up_cnt;
      imu_var.gyro[i] = (imu_var.gyro[i] * (up_cnt - 1) +
      	(imu.gyro[i] - imu_ave.gyro[i]) * (imu.gyro[i] - imu_ave.gyro[i])) / up_cnt;
    }
  } else {/*等于窗口长度*/
    for (int i = 0; i < 3; i++) {
      imu_ave.acce[i] = (imu_ave.acce[i] * (width - 1) + imu.acce[i]) / width;
      imu_ave.gyro[i] = (imu_ave.gyro[i] * (width - 1) + imu.gyro[i]) / width;

      imu_var.acce[i] = (imu_var.acce[i] * (width - 1) +
      	(imu.acce[i] - imu_ave.acce[i]) * (imu.acce[i] - imu_ave.acce[i])) / width;
      imu_var.gyro[i] = (imu_var.gyro[i] * (width - 1) +
      	(imu.gyro[i] - imu_ave.gyro[i]) * (imu.gyro[i] - imu_ave.gyro[i])) / width;
    }
  }
  imu_ave.gpst = imu.gpst;
  imu_pre = imu;
}
ImuData IMUSmooth::getSmoothedIMU() {
  return imu_ave;
}

double IMUSmooth::getStd() {
  return sqrt(imu_var.gyro[0] + imu_var.gyro[1] + imu_var.gyro[2]);
}

bool IMUSmooth::isStatic() {
  if (getStd() > static_std_threshold) {
    static_cnt = 0;
  } else {
    static_cnt++;
  }
  return static_cnt > static_width;
}