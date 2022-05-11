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
 * @param imu 前右下坐标系，增量格式
 */
void AlignMoving::Update(const ImuData &imu) {
  /*必须在静止时刻对准*/
  smooth.Update(imu);
  if (!smooth.isStatic()) {
	return;
  }
  auto smoothed_imu = smooth.getSmoothedIMU();
  Vec3d scale = Vec3d{option.imuPara.as_ini[0], option.imuPara.as_ini[1], option.imuPara.as_ini[2]},
	  bias = Vec3d{option.imuPara.ab_ini[0], option.imuPara.ab_ini[1], option.imuPara.ab_ini[2]},
	  acce = Vec3d{smoothed_imu.acce};
  Mat3d eye3 = Mat3d::Identity();
  double dt = 1.0 / option.d_rate;
  Mat3d scale_mat = scale.asDiagonal();
  acce = (eye3 - scale_mat) * (acce - bias * dt);
  nav.gpst = imu.gpst;
  Vec3d vm = acce.normalized();
  nav.atti[0] = asin(vm[1]) * (vm[2] > 0 ? 1 : -1);
  nav.atti[1] = -asin(vm[0]) * (vm[2] > 0 ? 1 : -1);
  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
  nav.Cbn = Convert::euler_to_dcm(nav.atti);
  flag_level_finished = true;
}

double AlignMoving::Update(const GnssData &gnss) {
/*  if (!(GnssCheck(gnss) > 0)) {
	return 0;
  }*/
  float v = 0;
  if (gnss_pre.pos_std[0] == 0) {
	gnss_pre = gnss;
	return -1;
  }
  Earth::Instance().Update(gnss.lat * _deg, gnss.height);
#if RUN_IN_STM32 != 1
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
#endif
	auto distance = Earth::Instance().distance(gnss.lat * _deg,
											   gnss.lon * _deg,
											   gnss_pre.lat * _deg,
											   gnss_pre.lon * _deg,
											   gnss.height,
											   gnss_pre.height);
	v = (float)distance.d;
	if (option.align_vel_threshold < distance.d and distance.d < 1e3) {
	  nav.vn[0] = distance.dn;
	  nav.vn[1] = distance.de;
	  nav.vn[2] = distance.dd;
	  nav.vel_std = {0.3, 0.3, 0.3};
	  nav.atti[2] = atan2(distance.de, distance.dn);
	  nav.att_std[0] = 0.1 * _deg;
	  nav.att_std[1] = 0.1 * _deg;
	  nav.att_std[2] = 1 * _deg;
	  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
	  nav.Cbn = Convert::euler_to_dcm(nav.atti);
	  flag_yaw_finished = true;
	}
#if RUN_IN_STM32 != 1
  }
  nav.gpst = gnss.gpst;
#endif
  nav.pos[0] = gnss.lat * _deg;
  nav.pos[1] = gnss.lon * _deg;
  nav.pos[2] = gnss.height;
  /*补偿杆臂影响*/
  Vec3d vdr = {1.0 / (Earth::Instance().RM(nav.pos[0]) + nav.pos[2]),
			   1.0 / ((Earth::Instance().RN(nav.pos[0]) + nav.pos[2]) * cos(nav.pos[0])),
			   -1
  };
  Vec3d lb = {option.lb_gnss[0], option.lb_gnss[1], option.lb_gnss[2]};
  Mat3d Dr = vdr.asDiagonal();
  nav.pos -= Dr * nav.Cbn * lb;
  /*补偿安z-axis影响*/
  Vec3d vec = Vec3d{0, 0, option.angle_bv[2]};
  Mat3d Cbv = Convert::euler_to_dcm(vec);
  nav.Cbn = nav.Cbn * Cbv;
  nav.atti = Convert::dcm_to_euler(nav.Cbn);
  nav.Qbn = Convert::euler_to_quaternion(nav.atti);
  auto ll = LatLon{nav.pos[0], nav.pos[1]};
  nav.Qne = Convert::lla_to_qne(ll);
  nav.Cne = Convert::lla_to_cne(ll);
  for (int i = 0; i < 3; i++) {
	nav.pos_std[i] = gnss.pos_std[i];
//	nav.att_std[i] = nav.vel_std[i] / gnss.pos_std[i];
//	nav.vel_std[i] = gnss.pos_std[i] + gnss_pre.pos_std[i];
	nav.gb[i] = option.imuPara.gb_ini[i];/*静止时候零偏作为对准之后的零偏 unit: ra */
	nav.ab[i] = option.imuPara.ab_ini[i];
	nav.gs[i] = option.imuPara.gs_ini[i];
	nav.as[i] = option.imuPara.as_ini[i];
  }
  nav.dvn = {0, 0, 0};
  nav.vf_kb = {0, 0, 0};
  nav.kd = option.odo_scale;
  nav.info.gnss_mode = gnss.mode;
  nav.info.sensors = SENSOR_GNSS | SENSOR_IMU;
  nav.week = gnss.week;
  gnss_pre = gnss;
  return v;
}

AlignMoving::AlignMoving(const Option &opt) : option(opt),
#if USE_INCREMENT == 1
											  smooth{1e-10, 2, 30}
#else
smooth{1.6e-4, 2, 10}
#endif
{

  nav.kd = opt.odo_scale;
  gnss_pre = {0};
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
