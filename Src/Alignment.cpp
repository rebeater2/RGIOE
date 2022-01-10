//
// Created by rebeater on 5/24/21.
//
#include "Define.h"
#include <InsCore.h>
#include "Alignment.h"
#include "matrix_lib.h"
#include "Convert.h"

/**
 * 水平调平
 * @param imu
 */
void AlignMoving::Update(const ImuData &imu) {
  /*必须在静止时刻对准*/
  smooth.Update(imu);
  auto smoothed_imu = smooth.getSmoothedIMU();
  nav.gpst = imu.gpst;
//    if (smooth.isStatic()) {
#if USE_INCREMENT == 1
  nav.atti[0] = asin(smoothed_imu.acce[0] * 200 / WGS84::Instance().g) * (smoothed_imu.acce[2] > 0 ? -1 : 1);
  nav.atti[1] = asin(smoothed_imu.acce[1] * 200 / WGS84::Instance().g) * (smoothed_imu.acce[2] > 0 ? 1 : -1);
#else
  /*用于加速度单位是1的场景*/
  nav.atti[0] = asin(aveimu.acce[1]) * (aveimu.acce[2] > 0 ? 1 : -1);
  nav.atti[1] = asin(aveimu.acce[0]) * (aveimu.acce[2] > 0 ? -1 : 1);
#endif
//  nav.att_std = {0.3 * _deg, 0.3 * _deg, 0.3 * _deg};
  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
  nav.Cbn = Convert::euler_to_dcm(nav.atti);
  flag_level_finished = true;
//    }
}

double AlignMoving::Update(const GnssData &gnss) {
/*  if (!(GnssCheck(gnss) > 0)) {
	return 0;
  }*/
  if (gnss_pre.pos_std[0] == 0) {
	gnss_pre = gnss;
	return 0;
  }
  WGS84::Instance().Update(gnss.lat * _deg, gnss.height);
  if (gnss.yaw >= 0 and gnss.yaw <= 360) {
	nav.atti[2] = gnss.yaw * _deg;
	nav.Qbn = Convert::euler_to_quaternion(nav.atti);
	nav.Cbn = Convert::euler_to_dcm(nav.atti);
	nav.att_std[2] = gnss.yaw_std * _deg;
	nav.vn[0] = 0;
	nav.vn[1] = 0;
	nav.vn[2] = 0;
	nav.vel_std = {0.4, 0.4, 0.4};
	flag_yaw_finished = true;
  } else {
	auto distance = WGS84::Instance().distance(gnss, gnss_pre);
	if (vel_threshold < distance.d and distance.d < 1e3) {
	  nav.vn[0] = distance.dn;
	  nav.vn[1] = distance.de;
	  nav.vn[2] = distance.dd;
	  nav.vel_std = {1.4, 1.4, 1.4};
	  nav.atti[2] = atan2(distance.de, distance.dn);
	  nav.att_std[0] = 5 * _deg;
	  nav.att_std[1] = 5 * _deg;
	  nav.att_std[2] = 5 * _deg;
	  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
	  nav.Cbn = Convert::euler_to_dcm(nav.atti);
	  flag_yaw_finished = true;
	}
  }
  nav.gpst = gnss.gpst;
  nav.pos[0] = gnss.lat * _deg;
  nav.pos[1] = gnss.lon * _deg;
  nav.pos[2] = gnss.height;
  auto ll = LatLon{nav.pos[0], nav.pos[1]};
  nav.Qne = Convert::lla_to_qne(ll);
  nav.Cne = Convert::lla_to_cne(ll);
  for (int i = 0; i < 3; i++) {
	nav.pos_std[i] = gnss.pos_std[i];
//	nav.att_std[i] = nav.vel_std[i] / gnss.pos_std[i];
	nav.vel_std[i] = gnss.pos_std[i] + gnss_pre.pos_std[i];
	nav.gb[i] = option.imuPara.gb_ini[i];/*静止时候零偏作为对准之后的零偏 unit: ra */
	nav.ab[i] = option.imuPara.ab_ini[i];
  }
  nav.dvn = {0, 0, 0};
  nav.vf_kb = {0, 0, 0};
  nav.gs = {0, 0, 0};
  nav.as = {0, 0, 0};
#if KD_IN_KALMAN_FILTER == 1
  nav.kd = option.kd_init;
#endif
  nav.info.gnss_mode = gnss.mode;
  nav.info.sensors = SENSOR_GNSS | SENSOR_IMU;
  nav.week = gnss.week;
  gnss_pre = gnss;
  return nav.vn[0] * nav.vn[0] + nav.vn[1] * nav.vn[1] + nav.vn[2] * nav.vn[2];
}

AlignMoving::AlignMoving(double vel_threshold, const Option &opt) : option(opt), vel_threshold(vel_threshold) {
#if KD_IN_KALMAN_FILTER == 1
  nav.kd = opt.kd_init;
#endif
  gnss_pre = {0, 0, 0,
			  0, 0, 0,
			  0,
			  0, 0, 0,
			  0, 0, 0,
			  0, 0, 0,
			  0, 0, 0, 0, 0, 0, 0,
			  {0, 0, 0, 0, 0, 0, 0, 0,}
  };
}

AlignBase::AlignBase() {
  nav.gpst = 0;
  auto zero = Vec3d::Zero();
  nav.pos = zero;/*n-frame position(lat,lon,alt) :d/d/m*/
  nav.vn = zero;/*n-frame velocity North East Down :m/a*/
  nav.atti = zero;/*attitude forward right down :rad*/

  nav.dvn = zero;/*n-frame velocity change :m/a*/
  nav.vf_kb = zero;

  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
  nav.Cbn = Convert::euler_to_dcm(nav.atti);
  auto ll = LatLon{nav.pos[0], nav.pos[1]};
  nav.Qne = Convert::lla_to_qne(ll);
  nav.Cne = Convert::lla_to_cne(ll);

  nav.gb = zero;/*gyroscope bias*/
  nav.ab = zero;/*accelerator bias*/
  nav.gs = zero;/*gyroscope sale factor error*/
  nav.as = zero;/*accelerator scale factor error*/

  nav.pos_std = zero;
  nav.vel_std = zero;
  nav.att_std = zero;
  flag_level_finished = false;
  flag_yaw_finished = false;
  nav.info.sensors = SensorType::SENSOR_IMU;
  nav.info.gnss_mode = GnssMode::INVALID;
  nav.week = 0;
}

NavOutput AlignBase::getPva() const {
  static NavOutput out;
  out.gpst = nav.gpst;
  out.lat = nav.pos[0] / _deg;
  out.lon = nav.pos[1] / _deg;
  out.height = (float)nav.pos[2];
  for (int i = 0; i < 3; i++) {
	out.vn[i] = (float)nav.vn[i];
	out.atti[i] = (float)(nav.atti[i] / _deg);
	out.gb[i] = (float)nav.gb[i];
	out.ab[i] = (float)nav.ab[i];
  }
  out.info = nav.info;
  return out;
}

NavEpoch AlignBase::getNavEpoch() const {
  return nav;
}

int AlignMoving::GnssCheck(const GnssData &gnss) {
  return 1;
};
