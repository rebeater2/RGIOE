//
// Created by rebeater on 2020/12/17.
//

#include "DataFusion.h"
#include "Recorder/RecorderType.h"

#define FLAG_POSITION 0b111U
#define FLAG_VELOCITY 0b111000U
#define FLAG_HEIGHT 0b1000000U

char CopyRight[] = "GNSS/INS/ODO Loosely-Coupled Program (1.01)\n"
                   "Copyright(c) 2019-2021, by Bao Linfeng, All rights reserved.\n"
                   "This Version is for Embedded and Real-time Application\n";

DataFusion::DataFusion() : Ins(),
#if USE_INCREMENT == 1
                           smooth{5e-9, 2, 10}
#else
smooth{1.6e-4, 2, 10}
#endif

{
    Q0.setZero();
    _timeUpdateIdx = 0;
    update_flag = 0x00;
}

/**
 * 初始化P,Q矩阵
 * @param ini_nav
 * @param opt
 */
void DataFusion::Initialize(const NavEpoch &ini_nav, const RgioeOption &RgioeOption) {
    /*initial P & Q0 */
    Vec1X Xd;
    Xd.setZero();
    this->opt = RgioeOption;
    InitializePva(ini_nav, opt.d_rate);
    nav = ini_nav;
    nav.kd = opt.odo_scale;
    Earth::Instance().Update(nav.pos[0], nav.pos[2]);
    MatXX P;
    P.setZero();
    P.block<3, 3>(0, 0) = ini_nav.pos_std.asDiagonal();
    P.block<3, 3>(3, 3) = ini_nav.vel_std.asDiagonal();
    P.block<3, 3>(6, 6) = ini_nav.att_std.asDiagonal();
    Vec3d temp = Vec3d{opt.imuPara.gb_std[0], opt.imuPara.gb_std[1], opt.imuPara.gb_std[2]};
    P.block<3, 3>(9, 9) = temp.asDiagonal();
    temp = Vec3d{opt.imuPara.ab_std[0], opt.imuPara.ab_std[1], opt.imuPara.ab_std[2]};
    P.block<3, 3>(12, 12) = temp.asDiagonal();
#if ESTIMATE_GYRO_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_GYRO_SCALE_FACTOR_SIZE; ++i)
      P(STATE_GYRO_SCALE_FACTOR_START + i, STATE_GYRO_SCALE_FACTOR_START + i) =
          opt.imuPara.gs_std[i] * opt.imuPara.gs_std[i];
#endif
#if ESTIMATE_ACCE_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_ACCE_SCALE_FACTOR_SIZE; ++i)
      P(STATE_ACCE_SCALE_FACTOR_START + i, STATE_ACCE_SCALE_FACTOR_START + i) =
          opt.imuPara.as_std[i] * opt.imuPara.as_std[i];
#endif
#if ESTIMATE_GNSS_LEVEL_ARM == 1
    double level_arm_std = 0.001;
    for (int i = 0; i < STATE_GNSS_LEVEL_ARM_SIZE; ++i) {
      P(STATE_GNSS_LEVEL_ARM_START + i, STATE_GNSS_LEVEL_ARM_START + i) = level_arm_std;
    }
#endif
#if ESTIMATE_ODOMETER_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_ODOMETER_SCALE_FACTOR_SIZE; ++i) {
      P(STATE_ODOMETER_SCALE_FACTOR_START + i, STATE_ODOMETER_SCALE_FACTOR_START + i) = 1e-9;
    }
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
#if ESTIMATE_GYRO_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_GYRO_SCALE_FACTOR_SIZE; ++i)
      Q0(STATE_GYRO_SCALE_FACTOR_START + i, STATE_GYRO_SCALE_FACTOR_START + i) =
          2 * opt.imuPara.gs_std[i] * opt.imuPara.gs_std[i] / opt.imuPara.gt_corr;
#endif
#if ESTIMATE_ACCE_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_ACCE_SCALE_FACTOR_SIZE; ++i)
      Q0(STATE_ACCE_SCALE_FACTOR_START + i, STATE_ACCE_SCALE_FACTOR_START + i) =
          2 * opt.imuPara.as_std[i] * opt.imuPara.as_std[i] / opt.imuPara.at_corr;
#endif
#if ESTIMATE_GNSS_LEVEL_ARM == 1
    for (int i = 0; i < STATE_GNSS_LEVEL_ARM_SIZE; ++i) {
      Q0(STATE_GNSS_LEVEL_ARM_START + i, STATE_GNSS_LEVEL_ARM_START + i) = 0;//2 * level_arm_std * level_arm_std / 36000;
    }
#endif
#if ESTIMATE_ODOMETER_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_ODOMETER_SCALE_FACTOR_SIZE; ++i) {
      Q0(STATE_ODOMETER_SCALE_FACTOR_START + i, STATE_ODOMETER_SCALE_FACTOR_START + i) =
          2 * opt.odo_scale_std * opt.odo_scale_std / 3600;
    }
#endif
/*initial kalman filter*/
    kf = KalmanFilter<STATE_CNT, double>(Xd, P);
    lb_gnss = Vec3d{opt.lb_gnss[0], opt.lb_gnss[1], opt.lb_gnss[2]};
    lb_wheel = Vec3d{opt.lb_wheel[0], opt.lb_wheel[1], opt.lb_wheel[2]};
    _timeUpdateIdx = 0;
    Cbv = Convert::euler_to_dcm({opt.angle_bv[0], opt.angle_bv[1], opt.angle_bv[2]});

}


/**
 * time update
 * @param imu : imu data
 * @return : 1 success 0 fail in time check
 */
int DataFusion::TimeUpdate(const RgioeImuData &imu) {
#if REAL_TIME_MODE != 1
    if (opt.enable_rts) {
        matp_posts.push_back(kf.P);
        Xds.push_back(kf.Xd);
        navs.push_back(nav);
    }
#endif
#ifdef ENABLE_FUSION_RECORDER
    recorder_msg_state_t state = CREATE_RECORDER_MSG(state);
    state.timestamp = nav.gpst;
    for(int i = 0; i < STATE_CNT;++i){
        state.data.xd[i] = kf.Xd[i];
    }
    CHECKSUM_RECORDER_CRC32(&state);
    Recorder::GetInstance().Record(&state);
#endif

/*    if (update_flag & FLAG_HEIGHT) {
        nav.pos[2] += kf.Xd[2];
        kf.Xd[2] = 0;
        update_flag &= ~FLAG_HEIGHT;
    }*/
    if (update_flag > 0) {
        _feedBack();
        kf.Reset();
        update_flag = 0;
    }

    smooth.Update(imu);
    ForwardMechanization(imu);
    MatXX phi = TransferMatrix(opt.imuPara);
    MatXX Q = 0.5 * (phi * Q0 + Q0 * phi.transpose()) * dt;
//    MatXd Q = 0.5 * (phi * Q0 * phi.transpose() + Q0) * dt;

    kf.Predict(phi, Q);
#if REAL_TIME_MODE != 1
    if (opt.enable_rts) {
        matp_pres.push_back(kf.P);
        matphis.push_back(phi);
    }
#endif
    _timeUpdateIdx++;
    if (_timeUpdateIdx % 8 and opt.zupt_enable) {
        if (smooth.isStatic()) {
            MeasureZeroVelocity();
            nav.info.sensors |= SENSOR_ZUPT;
        } else {
            nav.info.sensors &= ~SENSOR_ZUPT;
        }
    }
    /*NHC should be disabled when odometer is enable,since  */
    if (_timeUpdateIdx % 32 == 1 and opt.nhc_enable and !opt.odo_enable) {
        MeasureNHC();
        nav.info.sensors |= SENSOR_NHC;
    } else {
        nav.info.sensors &= ~SENSOR_NHC;
    }
#ifdef ENABLE_FUSION_RECORDER
    recorder_msg_kalman_t kalman = CREATE_RECORDER_MSG(kalman);
    kalman.timestamp = nav.gpst;
    for (int i = 0; i < 3; ++i) {
        kalman.data.acce_bias[i] = nav.ab[i] / _mGal;
        kalman.data.acce_scale[i] = nav.as[i] / _ppm;
        kalman.data.gyro_bias[i] = nav.gb[i] / _deg * _hour;
        kalman.data.gyro_scale[i] = nav.gs[i] / _ppm;
    }
    for (int i = 0; i < STATE_CNT; ++i) {
        kalman.data.matP[i] = kf.P(i, i);
    }
    CHECKSUM_RECORDER_CRC32(&kalman);
    Recorder::GetInstance().Record(&kalman);
    recorder_msg_imu_t imu_data = CREATE_RECORDER_MSG(imu);
    imu_data.timestamp = nav.gpst;
    for (int i = 0; i < 3; ++i) {
        imu_data.data.gyro[i] = imu.gyro[i];
        imu_data.data.acce[i] = imu.acce[i];
    }
    CHECKSUM_RECORDER_CRC32(&imu_data);
    Recorder::GetInstance().Record(&imu_data);
#endif
    return 0;
}

/**
 * Gnss Position Measure Update
 * @param pos
 * @param Rk
 * @return 1
 */
int DataFusion::MeasureUpdatePos(const Vec3d &pos, const Mat3d &Rk) {
    Mat3X H = _posH();
    Vec3d z = _posZ(pos);

#ifdef ENABLE_FUSION_RECORDER
    recorder_msg_meas_pos_t measPos = CREATE_RECORDER_MSG(meas_pos);
    measPos.timestamp = nav.gpst;
    for(int i = 0; i < 3;++i){
        measPos.data.z[i] = z[i];
        measPos.data.r[i] = Rk(i,i);
    }
    CHECKSUM_RECORDER_CRC32(&measPos);
    Recorder::GetInstance().Record(&measPos);
#endif
    kf.Update(H, z, Rk);
    update_flag |= FLAG_POSITION;
    return 0;
}

/**
 * GNSS数据检查，通过卫星数量、DOP、定位模式等决定是否使用当前观测，
 * @note 此函数默认使用GNSS数据并且不做检查，需要使用再外部重新定义即可
 * @param gnss GNSS数据
 * @return 0：不是用当前GNSS数据，1：使用当前GNSS数据
 */
int __attribute__((weak)) GnssCheck(const RgioeGnssData &gnss) {
    return gnss.mode == SPP or gnss.mode == RTK_FIX or gnss.mode == RTK_FLOAT or gnss.mode == RTK_DGPS;
}

int DataFusion::MeasureUpdatePos(const RgioeGnssData &gnssData) {
    if (GnssCheck(gnssData) > 0) {
        nav.info.sensors |= RgioeSensorType::SENSOR_GNSS;
        nav.info.gnss_mode = gnssData.mode;
        Vec3d pos(gnssData.lat * _deg, gnssData.lon * _deg, gnssData.height);
        Mat3d Rk = Mat3d::Zero();
        Rk(0, 0) = gnssData.pos_std[0] * gnssData.pos_std[0] * opt.gnss_std_scale;
        Rk(1, 1) = gnssData.pos_std[1] * gnssData.pos_std[1] * opt.gnss_std_scale;
        Rk(2, 2) = gnssData.pos_std[2] * gnssData.pos_std[2] * opt.gnss_std_scale;
        MeasureUpdatePos(pos, Rk);
        gnss_height = gnssData.height;
        base_height_is_set = 1;
    } else {
        nav.info.sensors &= ~RgioeSensorType::SENSOR_GNSS;
        nav.info.gnss_mode = GnssMode::INVALID;
    }
    return 0;
}

int DataFusion::MeasureUpdateVel(const Vec3d &vel) {
    nav.info.sensors |= RgioeSensorType::SENSOR_ODO;
    Mat3d Cnv = Cbv * nav.Cbn.transpose();
    Vec3d w_ib = _gyro_pre * opt.d_rate;
    Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
    Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));
    Mat3X H3 = Mat3X::Zero();
    H3.block<3, 3>(0, 3) = Cnv;
    H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
    H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
    Vec3d z = v_v - nav.kd * vel;
    if (opt.nhc_enable) {
        Mat3d R = Vec3d{opt.odo_std, opt.nhc_std[0], opt.nhc_std[1]}.asDiagonal();
        kf.Update<3>(H3, z, R * R);
        nav.info.sensors |= RgioeSensorType::SENSOR_NHC;
    } else {
        nav.info.sensors &= ~RgioeSensorType::SENSOR_NHC;
        Eigen::Matrix<double, 1, 1> R;
        R(0, 0) = opt.odo_std * opt.odo_std;
        kf.Update<1>(H3.block<1, STATE_CNT>(0, 0), z.block<1, 1>(0, 0), R);
    }

    update_flag |= FLAG_VELOCITY;/**/
    return 0;
}

int DataFusion::MeasureUpdateVel(const double &vel) {
    /* Vec3d w_ib = _gyro_pre * opt.d_rate;
     Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
     Vec3d v_v = nav.Cbn.transpose() * (nav.vn + Cbv * (omega_ie_n + omega_en_n));
     estimator_.Update(v_v, vel);*/
    return MeasureUpdateVel({vel, 0, 0});
}

KalmanFilter<STATE_CNT, double>::MatXX DataFusion::TransferMatrix(const ImuPara &para) {
    static MatXX phi;
    double g = Earth::Instance().g;
    double rm = Earth::Instance().RM(nav.pos[0]);
    double rn = Earth::Instance().RN(nav.pos[0]);
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
#if ESTIMATE_GYRO_SCALE_FACTOR == 1
    phi.block<3, 3>(STATE_GYRO_SCALE_FACTOR_START, STATE_GYRO_SCALE_FACTOR_START) = eye3 - eye3 * dt / para.at_corr;
  phi.block<3, 3>(9, STATE_GYRO_SCALE_FACTOR_START) = -nav.Cbn * _gyro_pre.asDiagonal();
#endif
#if ESTIMATE_ACCE_SCALE_FACTOR == 1
    phi.block<3, 3>(STATE_ACCE_SCALE_FACTOR_START, STATE_ACCE_SCALE_FACTOR_START) = eye3 - eye3 * dt / para.at_corr;
  phi.block<3, 3>(6, STATE_ACCE_SCALE_FACTOR_START) = nav.Cbn * _acce_pre.asDiagonal();
#endif
#if ESTIMATE_GNSS_LEVEL_ARM == 1
    for (int i = 0; i < STATE_GNSS_LEVEL_ARM_SIZE; ++i)
    phi(STATE_GNSS_LEVEL_ARM_START+i, STATE_GNSS_LEVEL_ARM_START+i) = 1.0 - dt / 36000;
#endif
#if ESTIMATE_ODOMETER_SCALE_FACTOR == 1
    for (int i = 0; i < STATE_ODOMETER_SCALE_FACTOR_SIZE; ++i)
    phi(STATE_ODOMETER_SCALE_FACTOR_START+i, STATE_ODOMETER_SCALE_FACTOR_START+i) = 1.0 - dt / 36000;
#endif
    return phi;
}

/**
 * feed back Modified Error Models
 * @return
 */
int DataFusion::_feedBack() {
    double lat = nav.pos[0];
    double h = nav.pos[2];
    double rn = Earth::Instance().RN(lat);
    double rm = Earth::Instance().RM(lat);
    Vec3d d_atti = Vec3d{kf.Xd[1] / (rn + h),
                         -kf.Xd[0] / (rm + h),
                         -kf.Xd[1] * tan(lat) / (rn + h)
    };
    Quad qnc = Convert::rv_to_quaternion(-d_atti);
    nav.Qne = (nav.Qne * qnc).normalized();
    LatLon ll = Convert::qne_to_lla(nav.Qne);
    nav.pos[0] = ll.latitude;
    nav.pos[1] = ll.longitude;
    nav.pos[2] = nav.pos[2] + kf.Xd[2];
    Mat3d Ccn = eye3 + Convert::skew(d_atti);
    nav.vn = Ccn * (nav.vn - Vec3d{kf.Xd[3], kf.Xd[4], kf.Xd[5]});
    Vec3d phi = Vec3d{kf.Xd[6], kf.Xd[7], kf.Xd[8]} + d_atti;
    Quad Qpn = Convert::rv_to_quaternion(phi);
    nav.Qbn = (Qpn * nav.Qbn).normalized();
    nav.Cbn = Convert::quaternion_to_dcm(nav.Qbn);
    nav.atti = Convert::dcm_to_euler(nav.Cbn);
    nav.gb += Vec3d{kf.Xd[9], kf.Xd[10], kf.Xd[11]};
    nav.ab += Vec3d{kf.Xd[12], kf.Xd[13], kf.Xd[14]};
#if ESTIMATE_GYRO_SCALE_FACTOR == 1
    nav.gs += kf.Xd.block<STATE_GYRO_SCALE_FACTOR_SIZE,1>(STATE_GYRO_SCALE_FACTOR_START,0);
#endif
#if ESTIMATE_ACCE_SCALE_FACTOR == 1
    nav.as += kf.Xd.block<STATE_ACCE_SCALE_FACTOR_SIZE,1>(STATE_ACCE_SCALE_FACTOR_START,0);
#endif
#if ESTIMATE_GNSS_LEVEL_ARM
    lb_gnss += kf.Xd.block<STATE_GNSS_LEVEL_ARM_SIZE, 1>(STATE_GNSS_LEVEL_ARM_START, 0);
#endif
#if ESTIMATE_ODOMETER_SCALE_FACTOR
    nav.kd += kf.Xd.block<STATE_ODOMETER_SCALE_FACTOR_SIZE, 1>(STATE_ODOMETER_SCALE_FACTOR_START, 0)(0);
#endif
    return 0;
}

KalmanFilter<STATE_CNT, double>::Mat3X DataFusion::_posH() const {
    Mat3X mat_h = Mat3X::Zero();
    mat_h.block<3, 3>(0, 0) = eye3;
    mat_h.block<3, 3>(0, 6) = Convert::skew(nav.Cbn * lb_gnss);
#if ESTIMATE_GNSS_LEVEL_ARM == 1
    Vec3d vdr = {1.0 / (Earth::Instance().RM(nav.pos[0]) + nav.pos[2]),
                 1.0 / ((Earth::Instance().RN(nav.pos[0]) + nav.pos[2]) * cos(nav.pos[0])),
                 -1
    };
    mat_h.block<3, STATE_GNSS_LEVEL_ARM_SIZE>(0, STATE_GNSS_LEVEL_ARM_START) = -(vdr.asDiagonal() * nav.Cbn);
#endif
    return mat_h;
}

KalmanFilter<STATE_CNT, double>::Mat3X DataFusion::_velH() const {
    Mat3d mat_h = Mat3d::Zero();
    mat_h.block<3, 3>(1, 3);
    Mat3d Cnv = Cbv * nav.Cbn.transpose();
    Vec3d w_ib = _gyro_pre / dt;
    Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
    Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));/*TODO odo*/
    Mat3X H3 = Mat3X::Zero();
    H3.block<3, 3>(0, 3) = Cnv;
    H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
    H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
#if ESTIMATE_GNSS_LEVEL_ARM == 1
    H3.block<STATE_GNSS_LEVEL_ARM_SIZE,STATE_GNSS_LEVEL_ARM_SIZE>(0,STATE_GNSS_LEVEL_ARM_START)=nav.Cbn;
#endif
    return H3;
}

/**
 * position measurement error
 * @param position in LatLon
 * @return
 */
Vec3d DataFusion::_posZ(const Vec3d &pos) {
    Vec3d re_ins = Convert::lla_to_xyz(nav.pos);
    Vec3d re_gnss = Convert::lla_to_xyz(pos);
    Mat3d cne = Convert::lla_to_cne({pos[0], pos[1]});
    Vec3d z = nav.Cne.transpose() * (re_ins - re_gnss) + nav.Cbn * lb_gnss;
    return z;
}

int DataFusion::MeasureNHC() {
    Mat3d Cnv = Cbv * nav.Cbn.transpose();
    Vec3d w_ib = _gyro_pre * opt.d_rate;
    Vec3d w_nb_b = w_ib - nav.Cbn.transpose() * (omega_ie_n + omega_en_n);
    Vec3d v_v = Cnv * nav.vn + Cbv * (w_nb_b.cross(lb_wheel));/*TODO odo*/
    Mat3X H3 = Mat3X::Zero();
    H3.block<3, 3>(0, 3) = Cnv;
    H3.block<3, 3>(0, 6) = -Cnv * Convert::skew(nav.vn);
    H3.block<3, 3>(0, 9) = -Cbv * Convert::skew(lb_wheel);
    Mat2d R = Vec2d{opt.nhc_std[0], opt.nhc_std[1]}.asDiagonal();
    R = R * R;
    Vec2d z = v_v.segment(1, 2);

    Mat2X H = H3.block<2, STATE_CNT>(1, 0);
    kf.Update(H, z, R);
    update_flag |= FLAG_VELOCITY;
    return 0;
}

int DataFusion::MeasureZeroVelocity() {
    /*零速观测*/
    Mat3X H = Mat3X::Zero();
    H.block<3, 3>(0, 3) = eye3;
    Vec3d z = nav.vn;
    Mat3d R = Mat3d::Zero();
    for (int i = 0; i < 3; i++) R(i, i) = opt.zupt_std * opt.zupt_std;
    kf.Update(H, z, R);
    update_flag |= FLAG_VELOCITY;
    /*ZUPT A*/
    if (!opt.zupta_enable) return 1;
    Vec1X HzuptA = Vec1X::Zero();
    HzuptA(0, 11) = 1;/*z轴零偏*/
    Eigen::Matrix<double, 1, 1> zputa{_gyro_pre[2] / dt};
    Eigen::Matrix<double, 1, 1> Rzupta{opt.zupta_std * opt.zupta_std};
    kf.Update(HzuptA, zputa, Rzupta);
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
float DataFusion::MeasureUpdateRelativeHeight(const double height) {

    if (base_height_is_set == 1) {
        p_height = height;
        nav.info.sensors &= ~SENSOR_HEIGHT;
    }
    if (base_height_is_set > 4) {
//        z = (nav.pos[2]) - (gnss_height + height - p_height);
        Vec3d posZ = _posZ({nav.pos[0], nav.pos[1], gnss_height + height - p_height});
        Mat3X posH = _posH();
        Vec1X H = posH.block<1, STATE_CNT>(2, 0);
        Eigen::Matrix<double, 1, 1> R{opt.zupta_std * opt.zupta_std};
        Eigen::Matrix<double, 1, 1> z{posZ[2]};
        kf.Update(H, z, R);
        update_flag |= FLAG_HEIGHT;
        nav.info.sensors |= SENSOR_HEIGHT;
    } else {
        nav.info.sensors &= ~SENSOR_HEIGHT;
    }
    base_height_is_set++;
    return (float) 0;
}

#if REAL_TIME_MODE != 1

//#include "glog/logging.h"
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

    VecX1 xdc = Xds.back();
    Xds.pop_back();

    nav = navs.back();
    navs.pop_back();

/*  LOG_FIRST_N(INFO,1) <<"matp:\n"<<matp;
  LOG_FIRST_N(INFO,1) <<"matphi:\n"<<matphi;
  LOG_FIRST_N(INFO,1) <<"matp1:\n"<<matp1;*/
    kf.RTSUpdate(matphi, matp);
/*    MatXX matA = matp * matphi.transpose() * matp1.inverse();
    kf.P = matp + matA * (kf.P - matp1) * matA.transpose();
    kf.Xd = xdc + matA * kf.Xd;*/
    _feedBack();
    return matp_posts.empty() or matphis.empty() or Xds.empty() or navs.empty();
}

#endif

NavOutput DataFusion::Output() const {
    Vec3d projpos = nav.pos;
    Vec3d projatti = nav.atti;
    if (opt.output_project_enable) {
        Vec3d atti = Vec3d{opt.atti_project[0], opt.atti_project[1], opt.atti_project[2]};
        Mat3d Cnx = Convert::euler_to_dcm(atti);
        projatti = Convert::dcm_to_euler(nav.Cbn * Cnx);
        Vec3d vdr = {1.0 / (Earth::Instance().RM(nav.pos[0]) + nav.pos[2]),
                     1.0 / ((Earth::Instance().RN(nav.pos[0]) + nav.pos[2]) * cos(nav.pos[0])),
                     -1
        };
        Vec3d outlb = {opt.pos_project[0], opt.pos_project[1], opt.pos_project[2]};
        projpos = nav.pos + vdr.asDiagonal() * nav.Cbn * outlb;
    }
    static NavOutput out;
    out.gpst = nav.gpst;
    out.lat = projpos[0] / _deg;
    out.lon = projpos[1] / _deg;
    out.height = (float) projpos[2];
    for (int i = 0; i < 3; i++) {
        out.vn[i] = (float) nav.vn[i];
        out.atti[i] = (float) (projatti[i] / _deg);
        out.gb[i] = (float) (nav.gb[i] / _deg * _hour);
        out.ab[i] = (float) (nav.ab[i] / _mGal);
        out.gs[i] = (float) (nav.gs[i] / _ppm);
        out.as[i] = (float) (nav.as[i] / _ppm);
    }
    out.kd = (float) nav.kd;
    out.info = nav.info;
    out.week = nav.week;
    for (int i = 0; i < 3; i++) {
        out.pos_std[i] = (float) sqrt(kf.P(0 + i, 0 + i));
        out.vn_std[i] = (float) sqrt(kf.P(3 + i, 3 + i));
        out.atti_std[i] = (float) sqrt(kf.P(6 + i, 6 + i));
    }
#ifdef ENABLE_FUSION_RECORDER
    static Vec3d first_pos = nav.pos;
    recorder_msg_result_t result = CREATE_RECORDER_MSG(result);
    result.timestamp = nav.gpst;
    Vec3d rpos = Earth::Instance().distance(nav.pos[0],nav.pos[1],first_pos[0],first_pos[1],nav.pos[2],first_pos[2]);
    for(int i = 0; i < 3; ++i){
        result.data.pos[i] = (float) rpos[i];
        result.data.vn[i] = (float) nav.vn[i];
        result.data.atti[i] = (float)nav.atti[i];
    }
    CHECKSUM_RECORDER_CRC32(&result);
    Recorder::GetInstance().Record(&result);
#endif
    return out;
}

