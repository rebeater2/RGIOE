//
// Created by rebeater on 5/24/21.
//
#include "RgioeDefine.h"
#include "InsCore.h"
#include "Alignment.h"
#include "RgioeMath.h"
#include "Convert.h"

#if ENABLE_FUSION_RECORDER

#include "Recorder/RecorderType.h"

#endif

/**
 * 水平调平
 * @param imu 前右下坐标系，增量格式
 */
void AlignMoving::Update(const RgioeImuData &imu) {
    /*必须在静止时刻对准*/
    nav.gpst = imu.gpst;
    smooth.Update(imu);
    if (flag_level_finished) {
        ahrs.Update(imu);
        return;
    }
    if (!smooth.isStatic()) {
        return;
    }
    auto smoothed_imu = smooth.getSmoothedIMU();
    Vec3d scale = Vec3d{option.imuPara.as_ini[0], option.imuPara.as_ini[1], option.imuPara.as_ini[2]},
            bias = Vec3d{option.imuPara.ab_ini[0], option.imuPara.ab_ini[1], option.imuPara.ab_ini[2]},
            acce = Vec3Hp{smoothed_imu.acce}.cast<RgioeFloatType>();
    Mat3d eye3 = Mat3d::Identity();
    double dt = 1.0 / option.d_rate;
    Mat3d scale_mat = scale.asDiagonal();
    acce = (eye3 - scale_mat) * (acce - bias * dt);
    nav.gb = Vec3d{smoothed_imu.gyro[0], smoothed_imu.gyro[1], smoothed_imu.gyro[2]} * option.d_rate;
    nav.gpst = imu.gpst;
    Vec3d vm = acce.normalized();
    nav.atti[0] = rgioe_asin(vm[1]) * (vm[2] > 0 ? 1 : -1);
    nav.atti[1] = -rgioe_asin(vm[0]) * (vm[2] > 0 ? 1 : -1);
    nav.Qbn = Convert::euler_to_quaternion(nav.atti);
    nav.Cbn = Convert::euler_to_dcm(nav.atti);
    flag_level_finished = true;
    ahrs.SetAtti(nav.Qbn);
    //LOG(INFO) <<"flag_level_finished imu:" << flag_level_finished << "  " << flag_yaw_finished;
    /*if (option.align_mode == ALIGN_USE_GIVEN) {
        nav.pos[0] = 0 * _deg;
        nav.pos[1] = 0 * _deg;
        nav.pos[2] = 0;
        nav.Qne = Convert::lla_to_qne({nav.pos[0], nav.pos[1]}),
                nav.pos_std = {0, 0, 0};
        nav.vn = {0, 0, 0};
        nav.vel_std = {0, 0, 0};
        nav.atti[2] = 0;
        nav.Qbn = Convert::euler_to_quaternion(nav.atti);
        nav.Cbn = Convert::euler_to_dcm(nav.atti);
        flag_yaw_finished = true;
    }*/
    //LOG(INFO) <<"flag_level_finished imu:" << flag_level_finished << "  " << flag_yaw_finished << " sys:"<<alignFinished();
}

double AlignMoving::Update(const RgioeGnssData &gnss) {
    if (gnss_pre.pos_std[0] == 0) {
        gnss_pre = gnss;
        return -1;
    }
    Earth::Instance().Update(gnss.lat * _deg, gnss.height);
#if RGIOE_ENABLE_DOUBLE_ANTENNA == 1
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
    auto vn_cur = Earth::Instance().distance(gnss.lat * _deg,
                                             gnss.lon * _deg,
                                             gnss_pre.lat * _deg,
                                             gnss_pre.lon * _deg,
                                             gnss.height,
                                             gnss_pre.height);
    auto v = (float) vn_cur.norm();
    double dgpst = gnss.gpst - gnss_pre.gpst;
    Vec3d acce = (vn_cur - vn_pre) / dgpst;/* 平均加速度 */
    vn_pre = vn_cur;
    if (option.align_vel_threshold < v and v < 1e3) {
        nav.vn = vn_cur + acce * dgpst / 2;
        nav.vel_std = {0.3, 0.3, 0.3};
        nav.atti[2] = rgioe_atan2(nav.vn[1], nav.vn[0]);
        nav.att_std[0] = 5 * _deg;
        nav.att_std[1] = 5 * _deg;
        nav.att_std[2] = 5 * _deg;
        nav.Qbn = Convert::euler_to_quaternion(nav.atti);
        nav.Cbn = Convert::euler_to_dcm(nav.atti);
        flag_yaw_finished = true;
    }
#if RGIOE_ENABLE_DOUBLE_ANTENNA == 1
    }
#endif
    nav.gpst = gnss.gpst;
    nav.pos[0] = gnss.lat * _deg;
    nav.pos[1] = gnss.lon * _deg;
    nav.pos[2] = gnss.height;
    /*补偿杆臂影响*/
    Vec3Hp vdr = {1.0 / (Earth::Instance().RM(nav.pos[0]) + (RgioeFloatType) nav.pos[2]),
                  1.0 / ((Earth::Instance().RN(nav.pos[0]) + (RgioeFloatType) nav.pos[2]) * cos(nav.pos[0])),
                  -1
    };
    Vec3d lb = {option.lb_gnss[0], option.lb_gnss[1], option.lb_gnss[2]};
    Mat3Hp Dr = vdr.asDiagonal();
    Vec3Hp delta_pos = Dr * (nav.Cbn * lb).cast<fp64>();
    Vec3Hp dpos = {
            delta_pos[0], delta_pos[1], delta_pos[2]
    };
    nav.pos = nav.pos - dpos;
    /* 补偿安z-axis影响 */
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
        nav.att_std[i] = nav.vel_std[i] / gnss.pos_std[i];

        nav.vel_std[i] = gnss.pos_std[i] + gnss_pre.pos_std[i];
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
    gnss_pre = gnss;
#if ENABLE_FUSION_RECORDER
    recorder_msg_align_t recorder = CREATE_RECORDER_MSG(align);
    recorder.timestamp = nav.gpst;
    for (int i = 0; i < 3; ++i) {
        recorder.data.atti[i] = nav.atti[i];
        recorder.data.vn[i] = nav.vn[i];
        recorder.data.pos[i] = (float) nav.pos[i];
    }
    recorder.data.level_align_finished = flag_level_finished;
    recorder.data.yaw_align_finished = flag_yaw_finished;
    recorder.data.v_norm = v;
    recorder.data.a_norm = smooth.getStd();
    recorder.data.is_staic = smooth.isStatic();
    Recorder::GetInstance().Record(&recorder);
#endif
    return v;
}

AlignMoving::AlignMoving(const RgioeOption &opt) :
        option(opt),
        smooth{1e-3, 200, 30},
        ahrs(AttiAhrsMethod_Mahony, 1.0 / opt.d_rate) {
    nav.gpst = 0;
    Vec3d zero = Vec3d::Zero();
    nav.pos = Vec3Hp::Zero();/*n-frame position(lat,lon,alt) :d/d/m*/
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
    nav.info.sensors = RgioeSensorType::SENSOR_IMU;
    nav.info.gnss_mode = GnssMode::INVALID;
    nav.week = 0;
    nav.kd = opt.odo_scale;
    gnss_pre = {0};
    flag_level_finished = false;
    flag_yaw_finished = false;

}

AlignBase::AlignBase() {
    nav.gpst = 0;
    Vec3d zero = Vec3d::Zero();
    nav.pos = Vec3Hp::Zero();/*n-frame position(lat,lon,alt) :d/d/m*/
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
    nav.info.sensors = RgioeSensorType::SENSOR_IMU;
    nav.info.gnss_mode = GnssMode::INVALID;
    nav.week = 0;
}

NavOutput AlignBase::getPva() const {
    static NavOutput out;
    out.gpst = nav.gpst;
    out.lat = nav.pos[0] / _deg;
    out.lon = nav.pos[1] / _deg;
    out.height = (float) nav.pos[2];
    for (int i = 0; i < 3; i++) {
        out.vn[i] = (float) nav.vn[i];
        out.atti[i] = (float) (nav.atti[i] / _deg);
        out.gb[i] = (float) nav.gb[i];
        out.ab[i] = (float) nav.ab[i];
    }
    out.info = nav.info;
    return out;
}

const NavEpoch &AlignBase::getNavEpoch() const {
    return nav;
}

int AlignMoving::GnssCheck(const RgioeGnssData &gnss) {
    return 1;
}

const bool AlignMoving::alignFinished() const { return flag_level_finished && flag_yaw_finished; }


int AlignMoving::SetOption(const RgioeOption &RgioeOption) {
    this->option = RgioeOption;
    return 0;
}

bool AlignMoving::levelFinished() const {
    return flag_level_finished;
}

void AlignStatic::Update(const RgioeImuData &imu) {
    AlignBase::Update(imu);
}
