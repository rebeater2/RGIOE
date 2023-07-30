/**
 * @file: LooselyCouple2020_cpp KalmanFilter.cpp
 * @author: rebeater
 * @function:  Implement of Kalman Filter
 * @date: 2020/11/28
 * @version: 1.0.0 @note first complement
 *
 **/


#include "KalmanFilter.hpp"

#if 0

template<int dim, typename fp>
void KalmanFilter<dim,fp>::Update(const Mat2Xd &H, const Vec2d &z, const Mat2d &R) {
  MatX2d K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
  xd = xd + K * (z - H * xd);
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * R * K.transpose();
}
#endif
