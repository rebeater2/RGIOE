//
// Created by rebeater on 2020/12/17.
//

#include "DataFusion.h"

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
  opt = {
	  {0.f, 0.f,
	   0 * _mGal, 0 * _mGal, 0 * _mGal,
	   0 * _deg / _hour, 0 * _deg / _hour, 0 * _deg / _hour,
	   0, 0, 0,
	   0, 0, 0,
	   0 * _mGal, 0 * _mGal, 0 * _mGal,
	   0 * _deg / _hour, 8 * _deg / _hour, 8 * _deg / _hour,
	   0 * _ppm, 0 * _ppm, 0 * _ppm,
	   0 * _ppm, 0 * _ppm, 0 * _ppm,
	   0 * _hour, 0 * _hour
	  },
	  {0, 0, 0, 0, 0, 0, 0,
	   0, 0, 0,
	   0, 0, 0,
	   0, 0, 0,
	   0, 0, 0},
	  0,
	  AlignMode::ALIGN_USE_GIVEN,
	  0, 0, 0, 0,
	  1, 0.3, 0.01, 0,
	  0.1, 0.3, -0.24,
	  0.2, 0.35,
	  0, 0, 0,
	  0, 0, 0,
	  0.5, 0.5, 0.9,
	  0.2, 0.2, 0.2,
	  0.3, 0.3, 0.3,
	  0.3, 0.2,
#if KD_IN_KALMAN_FILTER == 1
	  1.29, 0.3,
#endif
  };
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
  wgs84.Update(nav.pos[0], nav.pos[2]);
  P.setZero();
  P.block<3, 3>(0, 0) = ini_nav.pos_std.asDiagonal();
  P.block<3, 3>(3, 3) = ini_nav.vel_std.asDiagonal();
  P.block<3, 3>(6, 6) = ini_nav.att_std.asDiagonal();
  Vec3d temp = Vec3d{opt.imuPara.gb_std[0], opt.imuPara.gb_std[1], opt.imuPara.gb_std[2]};
  P.block<3, 3>(9, 9) = temp.asDiagonal();
  temp = Vec3d{opt.imuPara.ab_std[0], opt.imuPara.ab_std[1], opt.imuPara.ab_std[2]};
  P.block<3, 3>(12, 12) = temp.asDiagonal();
#if KD_IN_KALMAN_FILTER == 1
  float kd_std = opt.kd_std;
  P(15, 15) = kd_std;/*里程计比例因子*/
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
  Q0(15, 15) = 2 * kd_std * kd_std / opt.imuPara.gt_corr;
#endif
  lb_gnss = Vec3d{opt.lb_gnss[0], opt.lb_gnss[1], opt.lb_gnss[2]};
  lb_wheel = Vec3d{opt.lb_wheel[0], opt.lb_wheel[1], opt.lb_wheel[2]};
  _timeUpdateIdx = 0;
  Cbv = Convert::euler_to_dcm({opt.angle_bv[0], opt.angle_bv[1], opt.angle_bv[2]});
#if USE_OUTAGE == 1
  otg = Outage(opt.outage_start, opt.outage_stop, opt.outage_time, opt.outage_step);
#endif
}

/**
 * time update
 * @param imu : imu data
 * @return : 1 success 0 fail in time check
 */
int DataFusion::TimeUpdate(const ImuData &imu) {
  if (update_flag) {
	_feedBack();
	update_flag = 0;
  }
  smooth.Update(imu);
  ForwardMechanization(imu);
  MatXd phi = TransferMatrix(opt.imuPara);
  MatXd Q = 0.5 * (phi * Q0 + Q0 * phi.transpose()) * dt;
//    MatXd Q = 0.5 * (phi * Q0 * phi.transpose() + Q0) * dt;
  Predict(phi, Q);
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
#if USE_OUTAGE == 1
  if (opt.outage_enable and otg.IsOutage(gnssData.gpst)) {
	  /*outage mode*/
	  return -1;
  }
#endif
  if (GnssCheck(gnssData) > 0) {
	nav.info.sensors |= SensorType::SENSOR_GNSS;
	nav.info.gnss_mode = gnssData.mode;
	Vec3d pos(gnssData.lat * _deg, gnssData.lon * _deg, gnssData.height);
	Mat3d Rk = Mat3d::Zero();
	Rk(0, 0) = gnssData.pos_std[0] * gnssData.pos_std[0];
	Rk(1, 1) = gnssData.pos_std[1] * gnssData.pos_std[1];
	Rk(2, 2) = gnssData.pos_std[2] * gnssData.pos_std[2];
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
  Vec3d w_ib = _gyro_pre;
  Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
  Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));
  Mat3Xd H3 = Mat3Xd::Zero();
  H3.block<3, 3>(0, 3) = Cnv;
  H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
  H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);

#if KD_IN_KALMAN_FILTER == 1
  H3.block<3,1>(0,15)=vel;
  Vec3d z = v_v - nav.kd * vel;
#else
  Vec3d z = v_v - vel;
#endif
  auto R = Vec3d{0.1, 0.1, 0.1}.asDiagonal();/*TODO 参数，需要放到文件中*/
  Update(H3, z, R);
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
  double rn = wgs84.RN(lat);
  double rm = wgs84.RM(lat);
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
  Reset();
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

#if USE_OUTAGE == 1
Outage::Outage(int start, int stop, int outage, int step) : outage(outage) {
	/**/
	if ((start > stop and stop > 0) or outage < 0 or step < outage) {
		loge << "Outage parameter is invalid";
		flag_enable = false;
		return;
	}
	flag_enable = true;
	if (stop < 0) stop = start + 4000;
	for (int i = start; i < stop; i += step) {
		starts.push_back(i);
	}
	for (int &s:starts)
		logi << "outage time" << s;
}

/*!
 * 判断当前时间是否处于中断模式
 * */
bool Outage::IsOutage(double gpst) {
	if (!flag_enable) {
		return false;
	}
	for (auto &s:starts) {
		if (gpst <= s + outage) {
			return s <= gpst;
		}
	}
	return false;
}

Outage::Outage() {
	flag_enable = false;
	outage = 0;
}
#endif