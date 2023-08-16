/**
* @file DcmEstimator.h in LooselyCouple2020_cpp
* @author rebeater
* @comment Estimate DCM by Non-holo Constraint and odometer
* Create on 4/12/22 11:35 PM
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_INC_DCMESTIMATOR_H_
#define LOOSELYCOUPLE2020_CPP_INC_DCMESTIMATOR_H_
#include "RgioeMath.h"
class DcmEstimator {
 public:
  DcmEstimator();
  ~DcmEstimator();
  void Update(const Vec3d &vb);
  void Update(const Vec3d &vb, double vv_f);
  Mat3d GetDCM() const;
  Vec3d GetEulerAngles() const;
  double GetError()const;
  double GetScaleFactor()const;
 private:
  void CalcDcmBySVD(const Mat3d &W);
 private:
  Mat3d DCM_;
  Mat3d W_;
  double sum_norm_vf_;
  double scale_factor_;
  double error_;
};

#endif //LOOSELYCOUPLE2020_CPP_INC_DCMESTIMATOR_H_
