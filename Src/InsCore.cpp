/**
* @file: LooselyCouple2020_cpp ins_core.cpp
* @author: rebeater
* @function: 惯性导航核心函数 N系下机械编排的实现
* @date: 2020/11/10
* @version: 1.0.0
**/
#include "InsCore.h"
#include "Earth.h"

using namespace std;

NavEpoch makeNavEpoch(double gpst, Vec3Hp &pos, Vec3d &vn, Vec3d &atti) {
    Quad Qbn = Convert::euler_to_quaternion(atti);
    Mat3d Cbn = Convert::euler_to_dcm(atti);
    LatLon ll = LatLon{pos[0], pos[1]};
    QuadHp Qne = Convert::lla_to_qne(ll);
    Mat3Hp Cne = Convert::lla_to_cne(ll);
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

NavEpoch makeNavEpoch(NavOutput nav_, RgioeOption opt) {
    auto para = opt.imuPara;
    Vec3d atti = {nav_.atti[0] * _deg, nav_.atti[1] * _deg, nav_.atti[2] * _deg};
    auto vn = Vec3d{nav_.vn[0], nav_.vn[1], nav_.vn[2]};
    Quad Qbn = Convert::euler_to_quaternion(atti);
    Mat3d Cbn = Convert::euler_to_dcm(atti);
    auto pos = Vec3Hp{nav_.lat * _deg, nav_.lon * _deg, nav_.height};
    auto ll = LatLon{pos[0], pos[1]};
    QuadHp Qne = Convert::lla_to_qne(ll);
    Mat3Hp Cne = Convert::lla_to_cne(ll);
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
            opt.odo_scale,
            nav_.info,
            nav_.week
    };
    return nav;
}

int Ins::_velocity_update(const Vec3d &acce, const Vec3d &gyro) {
    Vec3d gn = {0, 0, (RgioeFloatType) Earth::Instance().g};
    Vec3d v_g_cor = (gn - (2 * omega_ie_n + omega_en_n).cross(nav.vn)) * dt;
    Vec3d zeta_mid = (omega_en_n + omega_ie_n) * dt;
    Vec3d vf_kb_k1 = acce + 0.5 * gyro.cross(acce) + (_gyro_pre.cross(acce) + _acce_pre.cross(gyro)) / 12.0;
    Vec3d vf_kb = (eye3 - 0.5 * Convert::skew(zeta_mid)) * nav.Cbn * vf_kb_k1;
    nav.dvn = vf_kb + v_g_cor;
    vn_mid = nav.vn + nav.dvn / 2.0;
    nav.vn = nav.vn + nav.dvn;
    nav.vf_kb = vf_kb / dt;
    omega_en_n = Earth::Instance().omega_en_n(vn_mid, pos_mid);
    return 0;
}

int Ins::_position_update() {
    Vec3d epsilon_k = Earth::Instance().omega_ie_e * dt;
    Quad q_e_e_delta = Convert::rv_to_quaternion(epsilon_k).conjugate();
    Vec3d zeta_k = (omega_ie_n + omega_en_n) * dt;
    Quad q_n_n_delta = Convert::rv_to_quaternion(zeta_k);
    QuadHp current_q_n_e = q_e_e_delta.cast<double>() * nav.Qne * q_n_n_delta.cast<double>();
    nav.Qne = current_q_n_e.normalized();
    // # 位置更新完毕，重新计算omega_en_n,omega_ie_e
    LatLon ll = Convert::qne_to_lla(nav.Qne);
    double h = nav.pos[2] - vn_mid[2] * dt;
    Vec3Hp pos = {ll.latitude, ll.longitude, h};
    nav.Cne = Convert::lla_to_cne(ll);
    /*x(k-1/2)=x(k-1)+0.5(x(k)- x(k-1))*/
    /*等价于 pos_mid = (pos+nav.pos)/2.0 */
    pos_mid = {
            pos[0] + 0.5 * (pos[0] - nav.pos[0]),
            pos[1] + 0.5 * (pos[1] - nav.pos[1]),
            0.5 * (pos[2] + nav.pos[2]),
    };
    nav.pos = pos;
    omega_ie_n = Earth::Instance().omega_ie_n(pos_mid[0]);
    omega_en_n = Earth::Instance().omega_en_n(vn_mid, pos_mid);
    return 0;
}

int Ins::_atti_update(const Vec3d &gyro) {
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
int Ins::ForwardMechanization(const RgioeImuData &imuData) {
#if USE_INCREMENT == 1
    Vec3d acce{(RgioeFloatType) imuData.acce[0], (RgioeFloatType) imuData.acce[1], (RgioeFloatType) imuData.acce[2]};
    Vec3d gyro{(RgioeFloatType) imuData.gyro[0], (RgioeFloatType) imuData.gyro[1], (RgioeFloatType) imuData.gyro[2]};
#else
    Vec3d acce{imuData.acce[0] * dt * Earth::Instance().g, imuData.acce[1] * dt * Earth::Instance().g, imuData.acce[2] * Earth::Instance().g * dt};
    Vec3d gyro{imuData.gyro[0] * dt, imuData.gyro[1] * dt, imuData.gyro[2] * dt};
#endif


    Vec3d acce_ = CompensateIMU(acce, nav.ab, nav.as);
    Vec3d gyro_ = CompensateIMU(gyro, nav.gb, nav.gs);
    ForwardMechanization(acce_, gyro_);
    nav.gpst = imuData.gpst;
    _acce_pre = acce_;
    _gyro_pre = gyro_;
    return 0;
}

int Ins::ForwardMechanization(const Vec3d &acce, const Vec3d &gyro) {
    /*外推一个周期*/
    _extrapolate();
    /*速度更新*/
    _velocity_update(acce, gyro);
    /*位置更新*/
    _position_update();
    /*姿态更新*/
    _atti_update(gyro);
    return 0;
}

Vec3d Ins::CompensateIMU(const Vec3d &imu, const Vec3d &bias, const Vec3d &scale) const {
    Mat3d scale_mat = scale.asDiagonal();
    return (eye3 - scale_mat) * (imu - bias * dt);
}

Ins::Ins() {
    eye3 = Eigen::Matrix<RgioeFloatType, 3, 3>::Identity();
    int d_rate = 100;
    dt = 1.0 / d_rate;
    _acce_pre.setZero();
    _gyro_pre.setZero();
    this->nav = NavEpoch{0, {0, 0, 0}};
}

Ins::~Ins() = default;

void Ins::InitializePva(const NavEpoch &nav_, const int d_rate) {
    eye3 = Eigen::Matrix<RgioeFloatType, 3, 3>::Identity();
    dt = 1.0 / d_rate;
    nav = nav_;
    _gyro_pre.setZero();
    _acce_pre.setZero();
    Earth::Instance().Update(nav.pos[0], nav.pos[2]);

}

void Ins::InitializePva(const NavEpoch &nav_, const RgioeImuData &imu) {
    eye3 = Eigen::Matrix<RgioeFloatType, 3, 3>::Identity();
    dt = 0.005;
    this->nav = nav_;
    _acce_pre = Vec3d{(RgioeFloatType) imu.acce[0], (RgioeFloatType) imu.acce[1], (RgioeFloatType) imu.acce[2]};
    _gyro_pre = Vec3d((RgioeFloatType) imu.gyro[0], (RgioeFloatType) imu.gyro[1], (RgioeFloatType) imu.gyro[2]);
    Earth::Instance().Update(nav.pos[0], nav.pos[2]);
}

void Ins::_extrapolate() {
    omega_en_n = Earth::Instance().omega_en_n(nav.vn, nav.pos); /*E 2.50*/
    omega_ie_n = Earth::Instance().omega_ie_n(nav.pos[0]);
    /*中间时刻的速度*/
    vn_mid = nav.vn + 0.5 * nav.dvn;
    /*中间时刻的高程*/
    double h_mid = nav.pos[2] - nav.vn[2] * dt / 2.0;
    /*中间时刻位置*/
    Vec3d zeta_mid = (omega_ie_n + omega_en_n) * dt / 2;
    Vec3d epsilon_mid = Earth::Instance().omega_ie_e * dt / 2;
    Quad q_nn_mid = Convert::rv_to_quaternion(zeta_mid);
    Quad q_ee_mid = Convert::rv_to_quaternion(epsilon_mid).conjugate();
    QuadHp q_ne_mid = q_ee_mid.cast<fp64>() * nav.Qne * q_nn_mid.cast<fp64>();
    LatLon lat_lon_mid = Convert::qne_to_lla(q_ne_mid.normalized());
    pos_mid = {lat_lon_mid.latitude, lat_lon_mid.longitude, h_mid};
    /*重新计算重力和角速度*/
    Earth::Instance().Update(pos_mid[0], pos_mid[2]);
    omega_en_n = Earth::Instance().omega_en_n(vn_mid, pos_mid); /*E 2.50*/
    omega_ie_n = Earth::Instance().omega_ie_n(pos_mid[0]);
}


NavOutput Ins::Output() const {
    static NavOutput out;
    out.gpst = nav.gpst;
    out.lat = nav.pos[0] / _deg;
    out.lon = nav.pos[1] / _deg;
    out.height = (float) nav.pos[2];

    for (int i = 0; i < 3; i++) {
        out.vn[i] = (float) nav.vn[i];
        out.atti[i] = (float) (nav.atti[i] / _deg);
        out.gb[i] = (float) (nav.gb[i] / _deg * _hour);
        out.ab[i] = (float) (nav.ab[i] / _mGal);
    }
    out.info = nav.info;
    out.week = nav.week;
    return out;
}
