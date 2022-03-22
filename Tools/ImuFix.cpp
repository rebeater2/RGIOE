/**
* @file ImuFix.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment IMU时间戳修复
* Create on 1/11/22 11:19 AM
* @version 1.0
**/
#include "FileIO.h"
#include "fmt/format.h"

void IMUFix(){
  std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/04/ADI51_220307_118100.raw.imd";
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
}
void OdoFix(){
  std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.veltxt";

  OdometerReader reader(filename);
  Velocity vel;
  reader.ReadNext(vel);
  ofstream out{filename + ".fix.veltxt"};
  while (reader.IsOk()) {
    reader.ReadNext(vel);
    vel.gpst -= 1.0;
   	out << fmt::format("{:.5f} {:.5f} {:.5f}\n",vel.gpst,vel.forward,vel.angular);
  }
}

int main(int argc, char **argv) {
  OdoFix();
  return 0;
}

