/**
* @file ZeroDetector.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment Zero Detect
* Create on 11/19/21 6:54 PM
* @version 1.0
**/

#include <iostream>
#include <fstream>
#include <NavStruct.h>
#include <DataFusion.h>
#include <fmt/format.h>
using namespace std;
ifstream &operator>>(ifstream &is, ImuData &imu) {

/*#if IMU_FRAME == 0 *//*前坐上*//*
  is.read((char *)&imu,sizeof(ImuData));
  //  is >> imu.gpst;
  //  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
  //  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
#else *//*zhu右前下坐标系*/
is >> imu.gpst;
is >> imu.gyro[1] >> imu.gyro[0] >> imu.gyro[2];
is >> imu.acce[1] >> imu.acce[0] >> imu.acce[2];
imu.gyro[2] *= (-1.0);
imu.acce[2] *= (-1.0);
//#endif
return is;
}

int main(int argc,char *argv[]){
  if(argc < 2){
    cout << "usage:ZeroDetect xxxx.imu"<<endl;
    return 1;
  }
  ifstream is(argv[1]);
  string s = argv[1];
  ofstream os(s+"_1.imustd");
  ImuData  imu;
  IMUSmooth smooth(0.005,5,3);
  if(!is.good()) cout << "no such file or directory";
  while(!is.eof() and is.good()){
    is >> imu;
    smooth.Update(imu);
    os << fmt::format("{:.4f} {:12.10f} {:d}\n",imu.gpst,smooth.getStd(),smooth.isStatic());
  }
  return 0;
}
