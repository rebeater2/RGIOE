/**
* @file: LooselyCouple2020_cpp ins_core.cpp
* @author: rebeater
* @function: 惯性导航核心函数 N系下机械编排的实现
* @date: 2020/11/10
* @version: 1.0.0
**/
#include <InsCore.h>
#include <WGS84.h>
#include <iomanip>
//#include <NavLog.h>

using namespace std;

NavEpoch makeNavEpoch(double gpst, Vec3d &pos, Vec3d &vn, Vec3d &atti) {
  Quad Qbn = Convert::euler_to_quaternion(atti);
  Mat3d Cbn = Convert::euler_to_dcm(atti);
  LatLon ll = LatLon{pos[0], pos[1]};
  Quad Qne = Convert::lla_to_qne(ll);
  Mat3d Cne = Convert::lla_to_cne(ll);
  NavEpoch nav{
	  gpst, pos, vn, atti,
	  Vec3d{0, 0, 0},
	  Vec3d{0, 0, 0},
	  Cbn, Cne, Qbn, Qne,
	  Vec3d{0, 0, 0},
	  Vec3d{0, 0, 0},
	  Vec3d{0, 0, 0},
	  Vec3d{0, 0, 0},
  };
  return nav;
}

NavEpoch makeNavEpoch(NavOutput nav_, Option opt) {
  auto para = opt.imuPara;
  Vec3d atti = {nav_.atti[0], nav_.atti[1], nav_.atti[2]};
  auto vn = Vec3d{nav_.vn[0], nav_.vn[1], nav_.vn[2]};
  Quad Qbn = Convert::euler_to_quaternion(atti);
  Mat3d Cbn = Convert::euler_to_dcm(atti);
  auto pos = Vec3d{nav_.lat, nav_.lon, nav_.height};
  auto ll = LatLon{pos[0], pos[1]};
  Quad Qne = Convert::lla_to_qne(ll);
  Mat3d Cne = Convert::lla_to_cne(ll);
  NavEpoch nav{
	  nav_.gpst, pos, vn, atti,
	  Vec3d{0, 0, 0},
	  Vec3d{0, 0, 0},
	  Cbn, Cne, Qbn, Qne,
	  Vec3d{para.gb_ini[0], para.gb_ini[1], para.gb_ini[2]},
	  Vec3d{para.ab_ini[0], para.ab_ini[1], para.ab_ini[2]},
	  Vec3d{para.gs_ini[0], para.gs_ini[1], para.gs_ini[2]},
	  Vec3d{para.as_ini[0], para.as_ini[1], para.as_ini[2]},
	  Vec3d{opt.pos_std[0], opt.pos_std[1], opt.pos_std[2]},
	  Vec3d{opt.vel_std[0], opt.vel_std[1], opt.vel_std[2]},
	  Vec3d{opt.atti_std[0], opt.atti_std[1], opt.atti_std[2]},
#if KD_IN_KALMAN_FILTER == 1
	  opt.kd_init,
#endif
	  nav_.info,
	  nav_.week
  };
  return nav;
}

std::ostream &operator<<(std::ostream &os, NavEpoch &nav) {
  os << nav.week << "\t" << fixed << setprecision(10) << nav.gpst << setprecision(12) << "\t" << nav.pos[0] / _deg
	 << " "
	 << nav.pos[1] / _deg << " " << setprecision(8) << nav.pos[2] << "\t"
	 << nav.vn[0] << " " << nav.vn[1] << " " << nav.vn[2] << "\t"
	 << nav.atti[0] / _deg << " " << nav.atti[1] / _deg << " " << nav.atti[2] / _deg;
  os << "\n" << "bias:" << nav.gb[0] << ' ' << nav.gb[1] << ' ' << nav.gb[2] << ' ' << nav.ab[0] << " " << nav.ab[1]
	 << ' ' << nav.ab[2] << '\n';
  os << "mode" << nav.info.gnss_mode << " " << nav.info.sensors;
  return os;

}

int Ins::_velocity_update(const Vec3d &acce, const Vec3d &gyro) {
  omega_en_n = wgs84.omega_en_n(vn_mid, pos_mid);
  omega_ie_n = wgs84.omega_ie_n(pos_mid[0]);

  Vec3d gn = {0, 0, wgs84.g};
  Vec3d v_g_cor = (gn - (2 * omega_ie_n + omega_en_n).cross(nav.vn)) * dt;
  Vec3d zeta_mid = (omega_en_n + omega_ie_n) * dt;
  Vec3d vf_kb_k1 = acce + 0.5 * gyro.cross(acce) + (_gyro_pre.cross(acce) + _acce_pre.cross(gyro)) / 12.0;
  Vec3d vf_kb = (eye3 - 0.5 * Convert::skew(zeta_mid)) * nav.Cbn * vf_kb_k1;
  nav.vn = nav.vn + vf_kb + v_g_cor;/*todo 更新omega_en_n?*/
  nav.dvn = vf_kb + v_g_cor;
  nav.vf_kb = vf_kb / dt;
  return 0;
}

int Ins::_position_update() {
  Vec3d epsilon_k = wgs84.omega_ie_e * dt;
  Quad q_e_e_delta = Convert::rv_to_quaternion(epsilon_k).conjugate();
  Vec3d zeta_k = (omega_ie_n + omega_en_n) * dt;
  Quad q_n_n_delta = Convert::rv_to_quaternion(zeta_k);
  Quad current_q_n_e = q_e_e_delta * nav.Qne * q_n_n_delta;
  nav.Qne = current_q_n_e.normalized();
  // # 位置更新完毕，重新计算omega_en_n,omega_ie_e
  LatLon ll = Convert::qne_to_lla(nav.Qne);
  vn_mid = nav.vn + nav.dvn / 2.0;
  double h = nav.pos[2] - vn_mid[2] * dt;
  nav.pos = {ll.latitude, ll.longitude, h};
  nav.Cne = Convert::lla_to_cne(ll);
  omega_ie_n = wgs84.omega_ie_n(ll.latitude);
  Vec3d temp = {nav.pos[0], nav.pos[1], pos_mid[2]};
  omega_en_n = wgs84.omega_en_n(vn_mid, temp);
  return 0;
}

int Ins::_atti_update(const Vec3d &gyro) {
//    ''' 位置更新 更新Qne'''
//    ''' 姿态更新更新Qbn'''
//# rotation vector
  Vec3d rv_b_b_delta = Convert::gyro_to_rv(gyro, _gyro_pre);
//#  b-frame quaternion
  Quad q_b_b_delta = Convert::rv_to_quaternion(rv_b_b_delta);
//# n-frame quaternion
  Vec3d zeta_k = (omega_ie_n + omega_en_n) * dt;
  Quad q_n_n_delta_skew = Convert::rv_to_quaternion(zeta_k).conjugate();
//# update quaternion
  nav.Qbn = (q_n_n_delta_skew * nav.Qbn * q_b_b_delta).normalized();
  nav.Cbn = Convert::quaternion_to_dcm(nav.Qbn);
  nav.atti = Convert::dcm_to_euler(nav.Cbn);
  return 0;
}

/**
 * 机械编排主函数
 * @param imuData
 * @return
 */
int Ins::ForwardMechanization(const ImuData &imuData) {
#if USE_INCREMENT == 1
  Vec3d acce{imuData.acce[0],imuData.acce[1],imuData.acce[2]};
  Vec3d gyro{imuData.gyro[0],imuData.gyro[1],imuData.gyro[2]};
#else
  Vec3d acce{imuData.acce[0] * dt * wgs84.g, imuData.acce[1] * dt * wgs84.g, imuData.acce[2] * wgs84.g * dt};
  Vec3d gyro{imuData.gyro[0] * dt, imuData.gyro[1] * dt, imuData.gyro[2] * dt};
#endif
//    dt = 0.005;//imuData.gpst - t_pre;
  t_pre = imuData.gpst;
  CompensateIMU(acce, nav.ab, nav.as);
  CompensateIMU(gyro, nav.gb, nav.gs);

  /*外插一个历元*/
  omega_en_n = wgs84.omega_en_n(nav.vn, nav.pos); /*E 2.50*/
  omega_ie_n = wgs84.omega_ie_n(nav.pos.x());
  /*外推一个周期*/
  pos_mid = _mid_pos();
  vn_mid = nav.vn + 0.5 * nav.dvn;
  /*速度更新*/
  _velocity_update(acce, gyro);
  /*位置更新*/
  _position_update();
  /*姿态更新*/
  _atti_update(gyro);
  nav.gpst = imuData.gpst;
  _acce_pre = acce;
  _gyro_pre = gyro;
  return 0;
}

void Ins::CompensateIMU(Vec3d &imu, const Vec3d &bias, const Vec3d &scale) const {
  Mat3d scale_mat = scale.asDiagonal();
  imu = (eye3 - scale_mat) * (imu - bias * dt);
}

Ins::Ins() {
  eye3 = Eigen::Matrix3d::Identity(3, 3);
  int d_rate = 100;
  dt = 1.0 / d_rate;
  this->nav = NavEpoch{0, {0, 0, 0}};
  t_pre = 0;
}

Ins::~Ins() = default;

void Ins::InitializePva(const NavEpoch &nav_, const int d_rate) {
  eye3 = Eigen::Matrix3d::Identity(3, 3);
  dt = 1.0 / d_rate;
  t_pre = nav.gpst;
  nav = nav_;
  wgs84.Update(nav.pos[0], nav.pos[2]);

}

void Ins::InitializePva(const NavEpoch &nav_, const ImuData &imu) {
  eye3 = Eigen::Matrix3d::Identity(3, 3);
  dt = 0.005;
  t_pre = nav.gpst;
  this->nav = nav_;
  _acce_pre = Vec3d(imu.acce);
  _gyro_pre = Vec3d(imu.gyro);
}

Vec3d Ins::_mid_pos() {
  double h_mid = nav.pos[2] - nav.vn[2] * dt / 2;
  Vec3d zeta_mid = (omega_ie_n + omega_en_n) * dt / 2;
  Vec3d epsilon_mid = wgs84.omega_ie_e * dt / 2;
  Quad q_nn_mid = Convert::rv_to_quaternion(zeta_mid);
  Quad q_ee_mid = Convert::rv_to_quaternion(epsilon_mid).conjugate();
  Quad q_ne_mid = q_ee_mid * nav.Qne * q_nn_mid;
  LatLon lat_lon_mid = Convert::qne_to_lla(q_ne_mid);
  pos_mid = {lat_lon_mid.latitude, lat_lon_mid.longitude, h_mid};
  wgs84.Update(pos_mid[0], pos_mid[2]);
  return pos_mid;
}

MatXd Ins::TransferMatrix(const ImuPara &para) {
  double g = wgs84.g;
  double rm = wgs84.RM(nav.pos[0]);
  double rn = wgs84.RN(nav.pos[0]);
  phi.setZero();
  phi.block<3, 3>(0, 0) = eye3 - Convert::skew(omega_en_n) * dt;
  phi.block<3, 3>(0, 3) = eye3 * dt;

  phi.block<3, 3>(3, 0) = Vec3d(-g / (rm + nav.pos[2]), -g / (rn + nav.pos[2]),
								2 * g / (sqrt(rm * rn) + nav.pos[2])).asDiagonal() * dt;

  Vec3d omega_in_n2 = omega_en_n + 2 * omega_ie_n;
  phi.block<3, 3>(3, 3) = eye3 - Convert::skew(omega_in_n2) * dt;
  phi.block<3, 3>(3, 6) = Convert::skew(nav.vf_kb) * dt;
  Vec3d omega_in_n = omega_en_n + omega_ie_n;
  phi.block<3, 3>(6, 6) = eye3 - Convert::skew(omega_in_n) * dt;
  phi.block<3, 3>(3, 12) = nav.Cbn * dt;
  phi.block<3, 3>(6, 9) = -nav.Cbn * dt;
  phi.block<3, 3>(9, 9) = eye3 - eye3 * dt / para.gt_corr;/*corr time*/
  phi.block<3, 3>(12, 12) = eye3 - eye3 * dt / para.at_corr;/*corr time*/
#if KD_IN_KALMAN_FILTER == 1
  phi(15, 15) = 1.0 - dt / para.gt_corr;/*TODO 这里暂时用了陀螺仪的相关时间作为里程计的相关时间*/
#endif
  return phi;
}

NavOutput Ins::Output() const {
  static NavOutput out;
  out.gpst = nav.gpst;
  out.lat = nav.pos[0] / _deg;
  out.lon = nav.pos[1] / _deg;
  out.height = (float)nav.pos[2];

  for (int i = 0; i < 3; i++) {
	out.vn[i] = (float)nav.vn[i];
	out.atti[i] = (float)(nav.atti[i] / _deg);
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
