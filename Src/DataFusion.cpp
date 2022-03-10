//
// Created by rebeater on 2020/12/17.
//

#include "DataFusion.h"
#include "iostream"
#define FLAG_POSITION 0b111U
#define FLAG_VELOCITY 0b111000U
//#define FLAG_YAW 0b100000000U
//#define VERSION 2.01
char CopyRight[] = "GNSS/INS/ODO Loosely-Coupled Program (1.01)\n"
				   "Copyright(c) 2019-2021, by Bao Linfeng, All rights reserved.\n"
				   "This Version is for Embedded and Real-time Application\n";

DataFusion::DataFusion() : Ins(), KalmanFilter() {
  P.setZero();
  Q0.setZero();
  /*这里的初始化参数在Initialize的时候会被覆盖掉，修改这里不会有任何影响*/
/*  Option default_option = default_option{
	  .imuPara={0, 0},
	  .init_epoch={0, 0},
	  .d_rate = 200,
	  .align_mode=AlignMode::ALIGN_USE_GIVEN,
	  .nhc_enable=false,
	  .zupt_enable=false,
	  .zupta_enable=false,
	  .zupt_std=0.00001,
	  .zupta_std=0.1 * _deg,
	  .lb_gnss={0, 0, 0},
	  .odo_std = 0.000001,
	  .lb_wheel={0, 0, 0},
	  .angle_bv={0, 0, 0},
	  .pos_std={0, 0, 0},
	  .vel_std={0, 0, 0},
	  .atti_std={10 * _deg, 10 * _deg, 10 * _deg},
	  .nhc_std={0.00001, 0.00001},
	  .kd_init=1.0,
	  .kd_std=0.001,
  };*/
  _timeUpdateIdx = 0;
  update_flag = 0x00;
}

/**
 * 初始化P,Q矩阵
 * @param ini_nav
 * @param opt
 */
void DataFusion::Initialize(const NavEpoch &ini_nav, const Option &option) {
  /*initial P & Q0 */
  xd.setZero();
  this->opt = option;
  InitializePva(ini_nav, opt.d_rate);
  nav = ini_nav;
  nav.kd = opt.kd_init;
  WGS84::Instance().Update(nav.pos[0], nav.pos[2]);
  P.setZero();
  P.block<3, 3>(0, 0) = ini_nav.pos_std.asDiagonal();
  P.block<3, 3>(3, 3) = ini_nav.vel_std.asDiagonal();
  P.block<3, 3>(6, 6) = ini_nav.att_std.asDiagonal();
  Vec3d temp = Vec3d{opt.imuPara.gb_std[0], opt.imuPara.gb_std[1], opt.imuPara.gb_std[2]};
  P.block<3, 3>(9, 9) = temp.asDiagonal();
  temp = Vec3d{opt.imuPara.ab_std[0], opt.imuPara.ab_std[1], opt.imuPara.ab_std[2]};
  P.block<3, 3>(12, 12) = temp.asDiagonal();
#if KD_IN_KALMAN_FILTER == 1
  P(15, 15) = opt.kd_std;/*里程计比例因子*/
#endif
  P = P * P;/*计算协方差矩阵*/
  Q0.setZero();
  Q0(3, 3) = opt.imuPara.vrw * opt.imuPara.vrw;
  Q0(4, 4) = opt.imuPara.vrw * opt.imuPara.vrw;
  Q0(5, 5) = opt.imuPara.vrw * opt.imuPara.vrw;
  Q0(6, 6) = opt.imuPara.arw * opt.imuPara.arw;
  Q0(7, 7) = opt.imuPara.arw * opt.imuPara.arw;
  Q0(8, 8) = opt.imuPara.arw * opt.imuPara.arw;

  Q0(9, 9) = 2 * opt.imuPara.gb_std[0] * opt.imuPara.gb_std[0] / opt.imuPara.gt_corr;
  Q0(10, 10) = 2 * opt.imuPara.gb_std[1] * opt.imuPara.gb_std[1] / opt.imuPara.gt_corr;
  Q0(11, 11) = 2 * opt.imuPara.gb_std[2] * opt.imuPara.gb_std[2] / opt.imuPara.gt_corr;

  Q0(12, 12) = 2 * opt.imuPara.ab_std[0] * opt.imuPara.ab_std[0] / opt.imuPara.at_corr;
  Q0(13, 13) = 2 * opt.imuPara.ab_std[1] * opt.imuPara.ab_std[1] / opt.imuPara.at_corr;
  Q0(14, 14) = 2 * opt.imuPara.ab_std[2] * opt.imuPara.ab_std[2] / opt.imuPara.at_corr;
#if KD_IN_KALMAN_FILTER == 1
  Q0(15, 15) = 2 *  opt.kd_std *  opt.kd_std / opt.imuPara.gt_corr;
#endif
  lb_gnss = Vec3d{opt.lb_gnss[0], opt.lb_gnss[1], opt.lb_gnss[2]};
  lb_wheel = Vec3d{opt.lb_wheel[0], opt.lb_wheel[1], opt.lb_wheel[2]};
  _timeUpdateIdx = 0;
  Cbv = Convert::euler_to_dcm({opt.angle_bv[0], opt.angle_bv[1], opt.angle_bv[2]}).transpose();

}

/**
 * time update
 * @param imu : imu data
 * @return : 1 success 0 fail in time check
 */
int DataFusion::TimeUpdate(const ImuData &imu) {
  if (opt.enable_rts) {
	matp_posts.push_back(P);
	Xds.push_back(xd);
	navs.push_back(nav);
  }
  if (update_flag) {
	_feedBack();
	Reset();
	update_flag = 0;
  }
  smooth.Update(imu);
  ForwardMechanization(imu);
  MatXd phi = TransferMatrix(opt.imuPara);
  MatXd Q = 0.5 * (phi * Q0 + Q0 * phi.transpose()) * dt;
//    MatXd Q = 0.5 * (phi * Q0 * phi.transpose() + Q0) * dt;

  Predict(phi, Q);
  if (opt.enable_rts) {
	matp_pres.push_back(P);
	matphis.push_back(phi);
  }

  _timeUpdateIdx++;

  if (_timeUpdateIdx % 16 and opt.zupt_enable) {
	if (smooth.isStatic()) {
	  MeasureZeroVelocity();
	  nav.info.sensors |= SENSOR_ZUPT;
	} else {
	  nav.info.sensors &= ~SENSOR_ZUPT;
	}
  }

  return 0;
}

/**
 * Gnss Position Measure Update
 * @param pos
 * @param Rk
 * @return 1
 */
int DataFusion::MeasureUpdatePos(const Vec3d &pos, const Mat3d &Rk) {
  Mat3Xd H = _posH();
  Vec3d z = _posZ(pos);
  Update(H, z, Rk);
  update_flag |= FLAG_POSITION;
  return 0;
}

/**
 * GNSS数据检查，通过卫星数量、DOP、定位模式等决定是否使用当前观测，
 * @note 此函数默认使用GNSS数据并且不做检查，需要使用再外部重新定义即可
 * @param gnss GNSS数据
 * @return 0：不是用当前GNSS数据，1：使用当前GNSS数据
 */
int __attribute__((weak)) GnssCheck(const GnssData &gnss) {
  return 1;
}
int DataFusion::MeasureUpdatePos(const GnssData &gnssData) {

  if (GnssCheck(gnssData) > 0) {
	nav.info.sensors |= SensorType::SENSOR_GNSS;
	nav.info.gnss_mode = gnssData.mode;
	Vec3d pos(gnssData.lat * _deg, gnssData.lon * _deg, gnssData.height);
	Mat3d Rk = Mat3d::Zero();
	Rk(0, 0) = gnssData.pos_std[0] * gnssData.pos_std[0] * opt.gnss_std_scale;
	Rk(1, 1) = gnssData.pos_std[1] * gnssData.pos_std[1] * opt.gnss_std_scale;
	Rk(2, 2) = gnssData.pos_std[2] * gnssData.pos_std[2] * opt.gnss_std_scale;
	MeasureUpdatePos(pos, Rk);
  } else {
	nav.info.sensors &= ~SensorType::SENSOR_GNSS;
	nav.info.gnss_mode = GnssMode::INVALID;
  }
  return 0;
}
int DataFusion::MeasureUpdateVel(const Vec3d &vel) {
  nav.info.sensors |= SensorType::SENSOR_ODO;
  Mat3d Cnv = Cbv * nav.Cbn.transpose();
  Vec3d w_ib = _gyro_pre * opt.d_rate;
  Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
  Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));
  Mat3Xd H3 = Mat3Xd::Zero();
  H3.block<3, 3>(0, 3) = Cnv;
  H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
  H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
#if KD_IN_KALMAN_FILTER == 1
  H3.block<3, 1>(0, 15) = vel;
#endif
  Vec3d z = v_v - nav.kd * vel;
  Mat3d R = Vec3d{opt.odo_std, opt.nhc_std[0], opt.nhc_std[1]}.asDiagonal();
  if(opt.nhc_enable){
    Update(H3, z, R*R);
  }else{
    Update(H3.block<STATE_CNT, 1>(0, 0), z[0], opt.odo_std*opt.odo_std);
  }

//  Update(H3, z, R);

  update_flag |= FLAG_VELOCITY;/**/
  return 0;
}

int DataFusion::MeasureUpdateVel(const double &vel) {
//  auto v = Vec3d;
  return MeasureUpdateVel({vel, 0, 0});
}

/**
 * feed back Modified Error Models
 * @return
 */
int DataFusion::_feedBack() {
  double lat = nav.pos[0];
  double h = nav.pos[2];
  double rn = WGS84::Instance().RN(lat);
  double rm = WGS84::Instance().RM(lat);
  Vec3d d_atti = Vec3d{xd[1] / (rn + h),
					   -xd[0] / (rm + h),
					   -xd[1] * tan(lat) / (rn + h)
  };
  Quad qnc = Convert::rv_to_quaternion(-d_atti);
  nav.Qne = (nav.Qne * qnc).normalized();
  LatLon ll = Convert::qne_to_lla(nav.Qne);
  nav.pos[0] = ll.latitude;
  nav.pos[1] = ll.longitude;
  nav.pos[2] = nav.pos[2] + xd[2];
  Mat3d Ccn = eye3 + Convert::skew(d_atti);
  nav.vn = Ccn * (nav.vn - Vec3d{xd[3], xd[4], xd[5]});
  Vec3d phi = Vec3d{xd[6], xd[7], xd[8]} + d_atti;
  Quad Qpn = Convert::rv_to_quaternion(phi);
  nav.Qbn = (Qpn * nav.Qbn).normalized();
  nav.Cbn = Convert::quaternion_to_dcm(nav.Qbn);
  nav.atti = Convert::dcm_to_euler(nav.Cbn);
  nav.gb += Vec3d{xd[9], xd[10], xd[11]};
  nav.ab += Vec3d{xd[12], xd[13], xd[14]};
#if KD_IN_KALMAN_FILTER == 1
  nav.kd += xd[15];
#endif

  return 0;
}

Mat3Xd DataFusion::_posH() const {
  Mat3Xd mat_h = Mat3Xd::Zero();
  mat_h.block<3, 3>(0, 0) = eye3;
  mat_h.block<3, 3>(0, 6) = Convert::skew(nav.Cbn * lb_gnss);
  return mat_h;
}

__attribute__((unused)) Mat3Xd DataFusion::_velH() const {
  Mat3d mat_h = Mat3d::Zero();
  mat_h.block<3, 3>(1, 3);
  Mat3d Cnv = Cbv * nav.Cbn.transpose();
  Vec3d w_ib = _gyro_pre / dt;
  Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
  Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));/*TODO odo*/
  Mat3Xd H3 = Mat3Xd::Zero();
  H3.block<3, 3>(0, 3) = Cnv;
  H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
  H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
  return H3;
}
/**
 * pos measurement
 * @param pos
 * @return
 */
Vec3d DataFusion::_posZ(const Vec3d &pos) {
  Vec3d re_ins = Convert::lla_to_xyz(nav.pos);
  Vec3d re_gnss = Convert::lla_to_xyz(pos);
  Mat3d cne = Convert::lla_to_cne({pos[0], pos[1]});
  Vec3d z = nav.Cne.transpose() * (re_ins - re_gnss) + nav.Cbn * lb_gnss;
  return z;
}

__attribute__((unused)) int DataFusion::MeasureNHC() {
  Mat3d Cnv = Cbv * nav.Cbn.transpose();
  Vec3d w_ib = _gyro_pre;
  Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
  Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));/*TODO odo*/
  Mat3Xd H3 = Mat3Xd::Zero();
  H3.block<3, 3>(0, 3) = Cnv;
  H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
  H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
  Mat2d R = Vec2d{opt.nhc_std[0], opt.nhc_std[1]}.asDiagonal();
  R = R * R;
  Vec2d z = v_v.segment(1, 2);

  Mat2Xd H = H3.block<2, STATE_CNT>(1, 0);
  Update(H, z, R);
  return 0;
}

int DataFusion::MeasureZeroVelocity() {
  /*零速观测*/

  Mat3Xd H = Mat3Xd::Zero();
  H.block<3, 3>(0, 3) = eye3;
  Vec3d z = nav.vn;
  Mat3d R = Mat3d::Zero();
  for (int i = 0; i < 3; i++) R(i, i) = 0.001;
  Update(H, z, R);
  /*ZUPT A*/
  Vec1Xd HzuptA = Vec1Xd::Zero();
  HzuptA(0, 11) = 1;/*z轴零偏*/
  double zputa = _gyro_pre[2];
  double Rzupta = 0.001;
  Update(HzuptA, zputa, Rzupta);
  update_flag |= FLAG_VELOCITY;
  return 0;
}
uint32_t DataFusion::EpochCounter() const {
  return _timeUpdateIdx;
}
/**
 * 高程观测更新
 * @param height：提供高程量测更新
 * $deltaZ = \hat\deltaH - delta H $
 * 利用相对高程变化的差计算高程误差，需要保存上时刻高程，和上时刻量测
 * @return 0
 */
int DataFusion::MeasureUpdateRelativeHeight(const double height) {
  if (p_height_ < -INT32_MAX + 1 or m_height_ < -INT32_MAX + 1) {
	m_height_ = height;
	p_height_ = nav.pos[2];
  } else {
	double z = (nav.pos[2] - p_height_) - (height - m_height_);
	m_height_ = height;
	p_height_ = nav.pos[2];
	Vec1Xd H = Vec1Xd::Zero();
	H[3] = 1;
	double R = 0.4;
	Update(H, z, R);
	update_flag |= SENSOR_HEIGHT;
  }
  return 0;
}
bool DataFusion::RtsUpdate() {
  if (matp_posts.empty() or matphis.empty() or Xds.empty() or navs.empty()) {
	return true;
  }
  auto matp = matp_posts.back();
  matp_posts.pop_back();

  auto matp1 = matp_pres.back();
  matp_pres.pop_back();

  auto matphi = matphis.back();
  matphis.pop_back();

  VecXd xdc = Xds.back();
  Xds.pop_back();

  nav = navs.back();
  navs.pop_back();

  auto matA = matp * matphi.transpose() * matp1.inverse();
  P = matp + matA * (P - matp1) * matA.transpose();
  xd = xdc + matA * xd;
  _feedBack();
  return matp_posts.empty() or matphis.empty() or Xds.empty() or navs.empty();
}
NavOutput DataFusion::Output() const {
  Vec3d projpos = nav.pos;
  Vec3d projatti = nav.atti;
  if (opt.output_project_enable) {
	Vec3d vdr = {1.0 / (WGS84::Instance().RM(nav.pos[0]) + nav.pos[2]),
				 1.0 / ((WGS84::Instance().RN(nav.pos[0]) + nav.pos[2]) * cos(nav.pos[0])),
				 -1
	};
	Vec3d outlb = {opt.pos_project[0], opt.pos_project[1], opt.pos_project[2]};
	projpos = nav.pos + vdr.asDiagonal() * nav.Cbn * outlb;
  }
  static NavOutput out;
  out.gpst = nav.gpst;
  out.lat = projpos[0] / _deg;
  out.lon = projpos[1] / _deg;
  out.height = (float)projpos[2];
  for (int i = 0; i < 3; i++) {
	out.vn[i] = (float)nav.vn[i];
	out.atti[i] = (float)(projatti[i] / _deg);
	out.gb[i] = (float)(nav.gb[i] / _deg * _hour);
	out.ab[i] = (float)(nav.ab[i] / _mGal);
  }
#if KD_IN_KALMAN_FILTER == 1
  out.kd =(float) nav.kd;
#endif
  out.info = nav.info;
  out.week = nav.week;
  return out;
}

