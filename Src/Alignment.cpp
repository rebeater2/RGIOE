//
// Created by rebeater on 5/24/21.
//

#include <InsCore.h>
#include <iostream>
#include "Alignment.h"
#include "matrix_lib.h"
#include "convert.h"


IMUSmooth::IMUSmooth() {
    imu_ave = {0, 0, 0, 0, 0, 0, 0};
    imu_var = {0, 0, 0, 0, 0, 0, 0};

    up_cnt = 0;
    static_cnt = 0;
}

/**
 * 滑动计算IMU的均值和方差
 * @param imu
 */
void IMUSmooth::Update(ImuData &imu) {
    up_cnt++;
    if (up_cnt < width) {/*小于窗口长度*/
        for (int i = 0; i < 3; i++) {
            imu_ave.acce[i] = (imu_ave.acce[i] * (up_cnt - 1) + imu.acce[i]) / up_cnt;
            imu_ave.gyro[i] = (imu_ave.gyro[i] * (up_cnt - 1) + imu.gyro[i]) / up_cnt;

            imu_var.acce[i] = (imu_var.acce[i] * (up_cnt - 1) +
                               (imu.acce[i] - imu_ave.acce[i]) * (imu.acce[i] - imu_ave.acce[i])) / up_cnt;
            imu_var.gyro[i] = (imu_var.gyro[i] * (up_cnt - 1) +
                               (imu.gyro[i] - imu_ave.gyro[i]) * (imu.gyro[i] - imu_ave.gyro[i])) / up_cnt;
        }
    } else {/*等于窗口长度*/
        for (int i = 0; i < 3; i++) {
            imu_ave.acce[i] = (imu_ave.acce[i] * (width - 1) + imu.acce[i]) / width;
            imu_ave.gyro[i] = (imu_ave.gyro[i] * (width - 1) + imu.gyro[i]) / width;
            imu_var.acce[i] = (imu_var.acce[i] * (width - 1) +
                               (imu.acce[i] - imu_ave.acce[i]) * (imu.acce[i] - imu_ave.acce[i])) / width;
            imu_var.gyro[i] = (imu_var.gyro[i] * (width - 1) +
                               (imu.gyro[i] - imu_ave.gyro[i]) * (imu.gyro[i] - imu_ave.gyro[i])) / width;
        }
    }
    imu_pre = imu;
}

ImuData IMUSmooth::getSmoothedIMU() {
    return imu_ave;
}

double IMUSmooth::getStd() {
    return sqrt(imu_var.gyro[0] + imu_var.gyro[1] + imu_var.gyro[2]);
}

bool IMUSmooth::isStatic() {
    if (getStd() > static_std_threshold) {
        static_cnt = 0;
    } else {
        static_cnt++;
    }
    return static_cnt > static_width;
}

/**
 * 水平调平
 * @param imu
 */
void AlignMoving::Update(ImuData &imu) {
    /*必须在静止时刻对准*/
    smooth.Update(imu);
    double g = wgs84.g;
    auto aveimu = smooth.getSmoothedIMU();
    nav.gpst = imu.gpst;
    if (smooth.isStatic()) {
#if USE_INCREMENT == 1
        nav.atti[0] = asin(aveimu.acce[1] * 200 / g) * (aveimu.acce[2] > 0 ? 1 : -1);
        nav.atti[1] = asin(aveimu.acce[0] * 200 / g) * (aveimu.acce[2] > 0 ? -1 : 1);
#else
        nav.atti[0] = asin(aveimu.acce[1] / g) * (aveimu.acce[2] > 0 ? 1 : -1);
        nav.atti[1] = asin(aveimu.acce[0] / g) * (aveimu.acce[2] > 0 ? -1 : 1);
#endif
        flag_level_finished = true;
    }
}

double AlignMoving::Update(GnssData &gnss) {
    wgs84.Update(gnss.lat, gnss.height);
    if (gnss_pre.mode != 1) {
        gnss_pre = gnss;
        return 0;
    }
    auto distance = wgs84.distance(gnss, gnss_pre);
    if (vel_threshold < distance.d and distance.d < 1e3) {
        nav.gpst = gnss.gpst;
        nav.pos[0] = gnss.lat * _deg;
        nav.pos[1] = gnss.lon * _deg;
        nav.pos[2] = gnss.height;
        nav.vn[0] = distance.dn;
        nav.vn[1] = distance.de;
        nav.vn[2] = distance.dd;
        nav.atti[2] = atan2(distance.de, distance.dn);
        nav.Qbn = convert::euler_to_quaternion(nav.atti);
        nav.Cbn = convert::euler_to_dcm(nav.atti);
        LatLon ll = LatLon{nav.pos[0], nav.pos[1]};
        nav.Qne = convert::lla_to_qne(ll);
        nav.Cne = convert::lla_to_cne(ll);

        for (int i = 0; i < 3; i++) {
            nav.pos_std[i] = gnss.pos_std[i];
            nav.vel_std[i] = gnss.pos_std[i] + gnss_pre.pos_std[i];
            nav.att_std[i] = (gnss.pos_std[i] + gnss_pre.pos_std[i]) / distance.d;
            nav.gb[i] = smooth.getSmoothedIMU().gyro[i];/*静止时候零偏作为对准之后的零偏 unit: ra */
            nav.ab[i] = 0;
        }

        flag_yaw_finished = true;
    }
    gnss_pre = gnss;
    return distance.d;
}

AlignMoving::AlignMoving(double vel_threshold) : vel_threshold(vel_threshold) {
    gnss_pre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
}

AlignBase::AlignBase() {
    nav.gpst = 0;
    auto zero = Eigen::Vector3d::Zero();
    nav.pos = zero;/*n-frame position(lat,lon,alt) :d/d/m*/
    nav.vn = zero;/*n-frame velocity North East Down :m/a*/
    nav.atti = zero;/*attitude forward right down :rad*/

    nav.dvn = zero;/*n-frame velocity change :m/a*/
    nav.vf_kb = zero;

    nav.Qbn = convert::euler_to_quaternion(nav.atti);
    nav.Cbn = convert::euler_to_dcm(nav.atti);
    LatLon ll = LatLon{nav.pos[0], nav.pos[1]};
    nav.Qne = convert::lla_to_qne(ll);
    nav.Cne = convert::lla_to_cne(ll);

    nav.gb = zero;/*gyroscope bias*/
    nav.ab = zero;/*accelerator bias*/
    nav.gs = zero;/*gyroscope sale factor error*/
    nav.as = zero;/*accelerator scale factor error*/

    nav.pos_std = zero;
    nav.vel_std = zero;
    nav.att_std = zero;
    flag_level_finished = false;
    flag_yaw_finished = false;
}

NavOutput AlignBase::getNav() {
    static NavOutput out;
    out.gpst = nav.gpst;
    for (int i = 0; i < 3; i++) {
        out.pos[i] = nav.pos[i];
        out.vn[i] = nav.vn[i];
        out.atti[i] = nav.atti[i];
        out.gb[i] = nav.gb[i];
        out.ab[i] = nav.ab[i];
    }
    return out;
}

NavEpoch &AlignBase::getNavEpoch() {
    return nav;
}
