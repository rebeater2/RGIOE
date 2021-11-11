/**
 * @file: LooselyCouple2020_cpp KalmanFilter.cpp
 * @author: rebeater
 * @function:  Implement of Kalman Filter
 * @date: 2020/11/28
 * @version: 1.0.0 @note first complement
 *
 **/


#include "KalmanFilter.h"
void KalmanFilter::Predict(const MatXd &PHI, const MatXd &Q) {
  xd = PHI * xd;
  P = PHI * P * PHI.transpose() + Q;
}
void KalmanFilter::Update(const Vec1Xd &H,  double z,  double R) {
  VecXd K = P * H.transpose() / (H * P * H.transpose() + R);
  xd = K * (z - H * xd);
  P = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H) * P;
}
/*全维卡尔曼更新：基本上用不着 */
void KalmanFilter::Update(const MatXd &H, const VecXd &z, const MatXd &R) {
/*  MatXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  xd = xd + K * (z - H * xd);
  P = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H) * P;*/
  VecXd xd_ = xd, Hi;
  MatXd P_ = P;
  VecXd K_;
  MatXd i_kh;
  double inno;
  for (int i = 0; i < z.rows(); i++) {
	Hi = H.row(i);
	VecXd temp1 = P_ * Hi;
	double temp2 = (Hi.transpose() * temp1)(0, 0) + R(i, i);
	K_ = temp1 / temp2;
	inno = (z.row(i) - Hi.transpose() * xd_)(0, 0);
	xd_ = xd_ + K_ * inno;
	i_kh = MatXd::Identity(STATE_CNT, STATE_CNT) - K_ * Hi.transpose();
	P_ = i_kh * P_ * i_kh.transpose() + K_ * R(i, i) * K_.transpose();
  }
  xd = xd_;
  P = P_;

}

/**
 * Kalman Update 3 columns
 * @param H
 * @param z
 * @param R
 */
/*3维卡尔曼更新*/
int counter = 0;
void KalmanFilter::Update(const Mat3Xd &H, const Vec3d &z, const Mat3d &R) {
#if SEQUENCED == 0
  counter++;
  MatX3d K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
  xd = xd + K * (z - H * xd);
//    xd =  K *z;
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * R * K.transpose();
#else
  VecXd xd_ = xd, temp1;
  VecX1d Hi;
  MatXd P_ = P;
  VecXd K_;
  MatXd i_kh;
  double inno, temp2;
  auto I = MatXd::Identity(STATE_CNT, STATE_CNT);
  for (int i = 0; i < z.rows(); i++) {
	Hi = H.row(i);
	temp1 = P_ * Hi.transpose();
	temp2 = (Hi * temp1)(0, 0) + R(i, i);
	K_ = temp1 / temp2;
	inno = (z.row(i) - Hi * xd_)(0, 0);
	xd_ = xd_ + K_ * inno;
	i_kh = I - K_ * Hi;
	P_ = i_kh * P_ * i_kh.transpose() + K_ * R(i, i) * K_.transpose();
  }
  xd = xd_;
  P = P_;
#endif
}
void KalmanFilter::Update(const Mat2Xd &H, const Vec2d &z, const Mat2d &R) {
  MatX2d K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
  xd = xd + K * (z - H * xd);
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * R * K.transpose();
}

void KalmanFilter::Reset() {
  /*设置Xd为0*/
  xd.setZero();
}

KalmanFilter::KalmanFilter() {
  xd = VecXd::Zero();
  P = MatXd::Zero();
}
SequencedKalmanFilter::SequencedKalmanFilter() {
  xd = VecXd::Zero();
  P = MatXd::Zero();
  /*TODO*/
}
SequencedKalmanFilter::~SequencedKalmanFilter() = default;
