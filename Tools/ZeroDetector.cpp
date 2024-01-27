/**
* @file ZeroDetector.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment Zero Detect
* Create on 11/19/21 6:54 PM
* @version 1.0
**/

#include <iostream>
#include <fstream>
#include "RgioeDataType.h"
#include "DataFusion.h"
#include <fmt/format.h>
#include "FileIO.h"
using std::cout;
using std::endl;
int main(int argc,char *argv[]){
  if(argc < 2){
    cout << "usage:ZeroDetect xxxx.imu"<<endl;
    return 1;
  }
  ifstream is(argv[1]);
  IMUReader reader{argv[1],IMU_FILE_IMUTXT,IMU_FRAME_RFU,true,125};
  string s = argv[1];
  ofstream os(s+"_1.imustd");
  ImuData  imu;
  IMUSmooth smooth(5e-9,2,10);
  if(!is.good()) cout << "no such file or directory";
  while(reader.IsOk()){
    reader.ReadNext(imu);
    smooth.Update(imu);
    os << fmt::format("{:.4f} {:12.10f} {:d}\n",imu.gpst,smooth.getStd(),smooth.isStatic());
  }
  return 0;
}
