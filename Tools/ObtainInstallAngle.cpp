/**
* @file ObtainInstallAngle.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 3/9/22 10:41 PM
* @version 1.0
**/

#include "matrix_lib.h"
#include "FileIO.h"
#include "Convert.h"
#include <glog/logging.h>
#include "Eigen/Dense"
#include <string>
#include "iostream"
using std::string;

int main() {
  google::InitGoogleLogging(".");
  google::LogToStderr();
  string nav_path = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482_cpt.nav";
  string out_path  = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482_cpt.angle";
  ofstream ofs(out_path);//  = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482_cpt.angle";
  NavReader reader(nav_path);
  double start_time = 122371;
  double end = 124130;
  NavOutput nav;
  if (!reader.ReadUntil(start_time, &nav)) {
	LOG(FATAL) << "please check the nav data and start time";
  };
  Mat3d P = 0.4 * Mat3d::Identity();
  Mat3d Q = 0 * Mat3d::Identity();
  Mat2d R = 100 * Mat2d::Identity();
  Vec3d theta = Vec3d::Zero();
  Mat3d eye3 = Mat3d::Identity();
  LOG(INFO) << "Calculate install angle...";
  while (reader.IsOk() and nav.gpst <= end ) {
	reader.ReadNext(nav);
//	LOG(INFO)<<nav;
	Mat3d Cbn = Convert::euler_to_dcm({nav.atti[0] * _deg, nav.atti[1] * _deg, nav.atti[2] * _deg});
	Vec3d vb = Cbn.transpose() * Vec3d{nav.vn[0], nav.vn[1], nav.vn[2]};
//	LOG(INFO) << "vb:" << vb.transpose();
	auto H = Convert::skew(-vb).block<2, 3>(1, 0);
	Vec2d Y = - vb.block<2, 1>(1, 0);
//	P = P + Q;
	Mat2d HPHR = H * P * H.transpose() + R;
	auto K = P * H.transpose() * HPHR.inverse();
	theta +=  K * (Y - H * theta);
	auto IKH = eye3 - K * H;
	P = IKH * P * IKH.transpose() + K * R * K.transpose();
	ofs << theta.transpose()<<'\n';
  }
  LOG(INFO) << "current theta:" << theta.transpose() * 180/_PI <<" at "<< nav.gpst;
}
