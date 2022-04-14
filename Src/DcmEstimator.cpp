/**
* @file DcmEstimator.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment Estimate DCM by Non-holo Constraint and odometer
* Create on 4/13/22 9:38 AM
* @version 1.0
**/
#include <glog/logging.h>
#include "DcmEstimator.h"
#include "Convert.h"
/**
 * Update W when odometer is not available
 * @param vb velocity of IMU in body-frame
 */
void DcmEstimator::Update(const Vec3d &vb) {
  W_.col(0) += (vb[0] * vb);
  CalcDcmBySVD(W_);
}
/**
 * Update W when odometer is available
 * @param vb velocity of IMU in body-frame
 * @param vv_f velocity of odometer velocity
 */
void DcmEstimator::Update(const Vec3d &vb, double vv_f) {
  sum_norm_vf_ += vv_f * vv_f;
  W_.col(0) += vv_f * vb;
  CalcDcmBySVD(W_);
}
/**@brief Get optimum DCM from v-frame to b-frame
 * @return DCM
 */
Mat3d DcmEstimator::GetDCM() const {
  return DCM_;
}
/** @brief
 * Get optimum install angles
 * @return
 */
Vec3d DcmEstimator::GetEulerAngles() const {
  return Convert::dcm_to_euler(DCM_);
}
double DcmEstimator::GetError() const {
  return error_;
}
void DcmEstimator::CalcDcmBySVD(const Mat3d &W) {
  Eigen::JacobiSVD<Mat3d> svd{W, Eigen::ComputeFullU | Eigen::ComputeFullV};
  const Mat3d &U = svd.matrixU();
  const Mat3d &V = svd.matrixV();
  DCM_ = V * U.transpose();
  scale_factor_ = svd.singularValues().x() / (sum_norm_vf_ + 1e-30);
}
DcmEstimator::DcmEstimator() {
  W_.setZero();
  DCM_ = Mat3d::Identity();
  scale_factor_ = 1.0;
  error_ = 0;
  sum_norm_vf_ = 0;
}
double DcmEstimator::GetScaleFactor() const {
  return scale_factor_;
}
DcmEstimator::~DcmEstimator() = default;

