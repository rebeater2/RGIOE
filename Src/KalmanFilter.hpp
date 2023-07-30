/**
* @file: LooselyCouple2020_cpp KalmanFilter.h
* @author: rebeater
* @function: a general Kalman filter
* @date: 2020/11/28 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
#define LOOSELYCOUPLE2020_CPP_KALMANFILTER_H



#include "Eigen/Dense"

#ifndef SEQUENCED
#define SEQUENCED 0
#endif

#define ENABLE_AKF 0


template<int dim, typename fp>
class KalmanFilter {
public:
    using MatXX = Eigen::Matrix<fp, dim, dim>;
    using MatX3 = Eigen::Matrix<fp, dim, 3>;
    using Mat3X = Eigen::Matrix<fp, 3, dim>;
    using Mat2X = Eigen::Matrix<fp, 2, dim>;
    using Vec1X = Eigen::Matrix<fp, 1, dim>;
    using VecX1 = Eigen::Matrix<fp, dim, 1>;


public:
    VecX1 Xd;
public:
    void Predict(const MatXX &PHI, const MatXX &Q);

/*    void Update(const Vec1Xd &H, double z, double R);

    void Update(const Vec1Xd &H, double z);

    void Update(const MatXd &H, const VecXd &Z, const MatXd &R);

    void Update(const Mat3Xd &H, const Vec3d &Z, const Mat3d &R);

    void Update(const Mat2Xd &H, const Vec2d &z, const Mat2d &R);*/

    template<int obs_dim>
    void Update(const Eigen::Matrix<fp,obs_dim, dim> &H,const Eigen::Matrix<fp,obs_dim,1> &Z,const Eigen::Matrix<fp,obs_dim,obs_dim> &R);

    void RTSUpdate(const MatXX &phi,const MatXX &matP);

    void Reset();

    KalmanFilter(Vec1X xd, MatXX P);//: xd(std::move(xd)), P(std::move(P))

    KalmanFilter();

    ~KalmanFilter();

public:
    MatXX P;
    MatXX Q0;
    double dk = 1;
    double b = 0.5;
    double rmin = 1e-4;
    double rmax = 0.01;
    double rk = 0.01;
};

template<int dim, typename fp>
void KalmanFilter<dim,fp>::Reset() {
    Xd.setZero();
}
template<int dim, typename fp>
KalmanFilter<dim,fp>::KalmanFilter() {
    Xd = Vec1X::Zero();
    P = MatXX::Zero();
}
template<int dim, typename fp>
KalmanFilter<dim,fp>::~KalmanFilter() {
}

template<int dim, typename fp>
template<int obs_dim>
void KalmanFilter<dim, fp>::Update(const Eigen::Matrix<fp, obs_dim, dim> &H, const Eigen::Matrix<fp, obs_dim, 1> &obs,
                                   const Eigen::Matrix<fp, obs_dim, obs_dim> &R) {
    Eigen::Matrix<fp, dim, obs_dim> K = P * H.transpose() * ((H * P * H.transpose() + R).inverse() + R);
    Xd = K * obs;
    MatXX temp = (Eigen::Matrix<fp,dim,dim>::Identity() - K * H);
    P = temp * P * temp.transpose() + K * R * K.transpose();
}
/**
 * RTS smooth algorithm TODO
 * @tparam dim the dimension of Kalman Filter
 * @tparam fp float type of the filter,such as float or double
 * @param phi transmit matrix
 * @param matP error state matrix
 */
template<int dim, typename fp>
void KalmanFilter<dim, fp>::RTSUpdate(const MatXX &phi,const MatXX &matP) {
    /*TODO*/
/*    auto matA = matp * matphi.transpose() * matp1.inverse();
    P = matp + matA * (P - matp1) * matA.transpose();
    Xd = xdc + matA * Xd;*/
}
template<int dim,typename fp>
void KalmanFilter<dim,fp>::Predict(const MatXX &PHI, const MatXX &Q) {
    Xd = PHI * Xd; /*xd保持为0*/
    P = PHI * P * PHI.transpose() + Q;
}

template<int dim, typename fp>
KalmanFilter<dim,fp>::KalmanFilter(Vec1X xd, MatXX P) : Xd(std::move(xd)), P(std::move(P)) {
}
#if 0
template<int dim, typename fp>
void KalmanFilter<dim,fp>::Update(const Vec1Xd &H, double z, double R) {
    VecXd K = P * H.transpose() / (H * P * H.transpose() + R);
    xd = K * z;
    MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
    P = temp * P * temp.transpose() + K * R * K.transpose();

}
template<int dim, typename fp>
void KalmanFilter<dim,fp>::Update(const Vec1Xd &H, double z) {
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

template<int dim, typename fp>
void KalmanFilter<dim,fp>::Update(const MatXd &H, const VecXd &z, const MatXd &R) {
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
int counter = 0;
template<int dim, typename fp>
void KalmanFilter<dim,fp>::Update(const Mat3Xd &H, const Vec3d &z, const Mat3d &R) {
    counter++;
#if SEQUENCED == 0
#if ENABLE_AKF == 1
    Rk = (1 - dk) * R + dk * (z * z.transpose() - H * P * H.transpose());
  dk = dk / (dk + b);
  assert(dk > 0 and dk < 1);
#else
    Rk = R;
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


}
#endif
#endif //LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
