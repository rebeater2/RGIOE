/**
* @file EstimateInstallAngular.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 4/13/22 9:38 AM
* @version 1.0
**/

#include <FileIO.h>
#include <glog/logging.h>
#include "DcmEstimator.h"
#include "Convert.h"
int main(int argc,char **argv){
  google::InitGoogleLogging(".");
  google::LogToStderr();
  LOG(INFO)<<"Loading "<<argv[0]<<std::endl;
  Mat3d W;
  W.setZero();
  W(0,0)=410236.687;
  W(1,0)=-16644.08431908;
  W(2,0)=5794.43237671;
  Eigen::JacobiSVD<Mat3d> svd{W,Eigen::ComputeFullU | Eigen::ComputeFullV};
//  svd.compute(W);
  const Mat3d &U = svd.matrixU();
  const Mat3d &V = svd.matrixV();
  LOG(INFO)<<"U = \n"<<svd.matrixU();
  LOG(INFO)<<"V = \n"<<V;
  LOG(INFO)<<"value = \n"<<svd.singularValues().transpose();
  auto R = V*U.transpose();
  LOG(INFO)<<"R = \n"<<svd.singularValues().transpose();
  LOG(INFO)<<"Angle = \n"<<Convert::dcm_to_euler(R).transpose()/_deg;

  LOG(INFO)<<"Error =\n"<<R*W;
  return 0;
}
