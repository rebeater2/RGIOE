/**
* @file: LooselyCouple2020_cpp KalmanFilter.h
* @author: rebeater
* @function: a general KalmanFiler
* @date: 2020/11/28 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
#define LOOSELYCOUPLE2020_CPP_KALMANFILTER_H




//#include <utility>

#include "matrix_lib.h"

#ifndef SEQUENCED
#define SEQUENCED 0
#endif

#define ENABLE_AKF 0
class KalmanFilter {
//  using Vec3x = Eigen::Matrix<double, 1, 3>;
//  Vec3x sf;
 public:
  VecXd xd;
  MatXd P;
  MatXd Q0;
  Mat3d Rk;
  double dk = 1;
  double b=0.9;
  double rmin = 1e-4;
  double rmax = 0.1;

 public:
  void Predict(const MatXd &PHI, const MatXd &Q);
  void Update(const Vec1Xd &H, double z, double R);
  void Update(const MatXd &H, const VecXd &Z, const MatXd &R);
  void Update(const Mat3Xd &H, const Vec3d &Z, const Mat3d &R);
  void Update(const Mat2Xd &H, const Vec2d &z, const Mat2d &R);
  void Reset();

  KalmanFilter(VecXd xd, MatXd P) : xd(std::move(xd)), P(std::move(P)) {}

  KalmanFilter();;
};
class SequencedKalmanFilter : public KalmanFilter {
 public:
  SequencedKalmanFilter();
  ~SequencedKalmanFilter();
 public:
//  void Update(double yk, double rk);
};
#endif //LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
