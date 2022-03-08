/**
* @file ImuFix.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment IMU时间戳修复
* Create on 1/11/22 11:19 AM
* @version 1.0
**/
#include "FileIO.h"
int main(int argc, char **argv) {
  std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.imd";
  int rate = 125;
  double dt = 1.0 / rate;
  IMUReader reader(filename, IMU_FILE_IMD, IMU_FRAME_FRD, true, rate);
  ImuData imu;
  reader.ReadNext(imu);
  ImuData pre = imu;
  ofstream out{filename + ".fix.imd",ios::binary};
  while (reader.IsOk()) {
	reader.ReadNext(imu);
	imu.gpst -= 1.0;
	out.write((const char *)&imu,sizeof imu);
  }
  return 0;
}

