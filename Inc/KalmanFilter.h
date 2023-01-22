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

/*保存P和z,for debug*/
#ifndef KALMAN_DEBUG
#define KALMAN_DEBUG 0
#endif

#define ENABLE_AKF 0
#if KALMAN_DEBUG == 1
#include <fstream>
#endif

class KalmanFilter {
 protected:
  MatXd P;
  MatXd Q0;
  Mat3d Rk;
  double dk = 1;
  double b=0.5;
  double rmin = 1e-4;
  double rmax = 0.01;
	double rk=0.01;
 public:
  VecXd xd;
 public:
  void Predict(const MatXd &PHI, const MatXd &Q);
  void Update(const Vec1Xd &H, double z, double R);
  void Update(const Vec1Xd &H, double z);
  void Update(const MatXd &H, const VecXd &Z, const MatXd &R);
  void Update(const Mat3Xd &H, const Vec3d &Z, const Mat3d &R);
  void Update(const Mat2Xd &H, const Vec2d &z, const Mat2d &R);
  void Reset();

  KalmanFilter(VecXd xd, MatXd P) ;//: xd(std::move(xd)), P(std::move(P))

  KalmanFilter();

  ~KalmanFilter();
#if KALMAN_DEBUG == 1
 private:
  std::ofstream ofs;
#endif
};
class SequencedKalmanFilter : public KalmanFilter {
 public:
  SequencedKalmanFilter();
  ~SequencedKalmanFilter();
 public:
//  void Update(double yk, double rk);
};
#endif //LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
