/**
 * @file: LooselyCouple2020_cpp KalmanFilter.cpp
 * @author: rebeater
 * @function:  Implement of Kalman Filter
 * @date: 2020/11/28
 * @version: 1.0.0 @note first complement
 *
 **/


#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(VecXd xd, MatXd P) : xd(std::move(xd)), P(std::move(P)) {
#if KALMAN_DEBUG == 1
  ofs.open("./debug_info.txt");
#endif
}
int predict_cnt = 0;
void KalmanFilter::Predict(const MatXd &PHI, const MatXd &Q) {
  xd = PHI * xd; /*xd保持为0*/
  P = PHI * P * PHI.transpose() + Q;
}
void KalmanFilter::Update(const Vec1Xd &H, double z, double R) {
  VecXd K = P * H.transpose() / (H * P * H.transpose() + R);
  xd = K * z;
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * R * K.transpose();

}
void KalmanFilter::Update(const Vec1Xd &H, double z) {
  double inno = z * z - H * P * H.transpose();
  rk = (1 - dk) * rk + rk * (z * z - H * P * H.transpose());
  if (inno < rmin) {
	rk = (1 - dk) * rk + dk * rmin;
  } else if (inno > rmax) {
	rk = rmax;
  } else {
	rk = (1 - dk) * rk + dk * inno;
  }
  dk = dk / (dk + b);
  VecXd K = P * H.transpose() / (H * P * H.transpose() + rk);
  xd = K * z;
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * rk * K.transpose();
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
  counter++;
#if SEQUENCED == 0
#if ENABLE_AKF == 1
  Rk = (1 - dk) * R + dk * (z * z.transpose() - H * P * H.transpose());
  dk = dk / (dk + b);
  assert(dk > 0 and dk < 1);
#else
  Rk = R+ 0.005*(z.transpose()*z).x() * Mat3d::Identity();
  Rk(2,2)=0.01 * R(2,2);
#endif
  MatX3d K = P * H.transpose() * ((H * P * H.transpose() + Rk).inverse() + Rk);
//  xd = xd + K * (z - H * xd);
  xd = K * z;
  MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
  P = temp * P * temp.transpose() + K * R * K.transpose();
#else
  VecXd xd_ = xd, temp1;
  Vec1Xd Hi;
  MatXd P_ = P;
  VecXd K_;
  MatXd i_kh;
  double inno, temp2, rk;
  auto I = MatXd::Identity(STATE_CNT, STATE_CNT);
  for (int i = 0; i < z.rows(); i++) {
	Hi = H.row(i);
	inno = (z.row(i) - Hi * xd_)(0, 0);
	/*constrain Rk*/
#if ENABLE_AKF == 1
	if (inno < rmin) {
	  rk = (1 - dk) * R(i, i) + dk * rmin;
	} else if (inno > rmax) {
	  rk = rmax;
	} else {
	  rk = (1 - dk) * R(i, i) + dk * inno;
	}
	dk = dk / (dk + b);
#else
	rk = R(i,i);
#endif
	temp1 = P_ * Hi.transpose();
	temp2 = (Hi * temp1)(0, 0) + rk;
	K_ = temp1 / temp2;

	xd_ = xd_ + K_ * inno;
	i_kh = I - K_ * Hi;
	P_ = i_kh * P_ * i_kh.transpose() + K_ * rk * K_.transpose();
  }
  xd = xd_;
  P = P_;
#endif
#if KALMAN_DEBUG == 1
  ofs << "P: " << P.diagonal().transpose() <<" "<<counter<<" "<< '\n';
  ofs << "Z: " << z.transpose() << '\n';
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
  #if KALMAN_DEBUG == 1
  ofs.open("./debug_info.txt");
#endif
}
KalmanFilter::~KalmanFilter() {
#if KALMAN_DEBUG == 1
  ofs.close();
#endif
}
SequencedKalmanFilter::SequencedKalmanFilter() {
  xd = VecXd::Zero();
  P = MatXd::Zero();
  /*TODO*/
}
SequencedKalmanFilter::~SequencedKalmanFilter() = default;
