/**
* @file ImuFix.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 1/11/22 11:19 AM
* @version 1.0
**/
#include "FileIO.h"
int main(int argc, char **argv) {
  std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220106/1/RAW_220107_455721.RAW.imutxt";
  int rate = 200;
  double dt = 1.0 / rate;
  IMUReader reader(filename, IMU_FILE_IMUTXT, IMU_FRAME_FRD, true, rate);
  ImuData imu;
  reader.ReadNext(imu);
  ImuData pre = imu;
  ofstream out{filename + "fix.imutxt"};
  while (reader.IsOk()) {
	reader.ReadNext(imu);
	while (imu.gpst - pre.gpst > dt * 1.5) {
	  /*用旧的数据填补缺失数据*/
	  /*其实这么做不科学*/
	  pre.gpst += dt;
	  out << pre << '\n';
	}
	out << imu << '\n';
	pre = imu;
  }
  return 0;
}

