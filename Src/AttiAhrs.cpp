/******************************************************************************
 * RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter
 * Copyright (C) 2024                                                         *
 * Author : rebeater                                                          *
 * Contact : rebeater@qq.com                                                  *
 ******************************************************************************/

#include "RgioeMath.h"
#include "AttiAhrs.h"
#include "Convert.h"
#if ENABLE_FUSION_RECORDER
#include "Recorder.h"
#endif

AttiAhrs::AttiAhrs(AttiAhrsMethod_t method, RgioeFloatType dt): ts(dt),method(method){
    RgioeFloatType beta = 2.146f / 100.0f;
    Kp = 2.0f * beta, Ki = beta * beta;
    qnb = Convert::euler_to_quaternion({0,0,0});
    Cnb = Convert::quaternion_to_dcm(qnb);
    exyzInt.setZero();
    ebMax = Vec3d{1,1,1}* _deg;
    tk = 0.0;
}
inline static double normXY(const Vec3d &vec) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}
/**
 *
 * @param wm
 * @param vm
 */
void AttiAhrs::Update(const Vec3d &wm, const Vec3d &vm) {
    double nm;
    Vec3d mag = Vec3d::Zero();
    Vec3d acc0, mag0, exyz, bxyz, wxyz;
    const Vec3d O31 = Vec3d ::Zero();
    nm = vm.norm() / ts;  // f (in m/s^2)
    acc0 = nm>0.5 ? vm/(nm*ts) : O31;
    nm = mag.norm();    // mag (in Gauss)
    if (nm > 0.1) {
        mag0 = mag / nm;
        bxyz = Cnb * mag0;
        bxyz[0] = normXY(bxyz);
        bxyz[1] = 0.0;
        wxyz = (Cnb.conjugate()) * bxyz;
    } else {
        mag0.setZero();
        wxyz.setZero();
    }
    exyz = Cnb.row(2).transpose().cross(acc0) + wxyz.cross(mag0);
//  exyz = *((Vec3d *)&Cnb.e20) * acc0 + wxyz * mag0;
    exyzInt += exyz * (Ki * ts);
    qnb *= Convert::rv_to_quaternion(wm - (Kp * exyz + exyzInt) * ts);
    qnb.normalize();
    Cnb = Convert::quaternion_to_dcm(qnb);
    tk += ts;
    if (exyzInt[0] > ebMax[0]) exyzInt[0] = ebMax[0]; else if (exyzInt[0] < -ebMax[0]) exyzInt[0] = -ebMax[0];
    if (exyzInt[1] > ebMax[1]) exyzInt[1] = ebMax[1]; else if (exyzInt[1] < -ebMax[1]) exyzInt[1] = -ebMax[1];
    if (exyzInt[2] > ebMax[2]) exyzInt[2] = ebMax[2]; else if (exyzInt[2] < -ebMax[2]) exyzInt[2] = -ebMax[2];
}

void AttiAhrs::Update(const Vec3d &gyro, const Vec3d &acc, const Vec3d &mag) {
    double nm;
    Vec3d acc0, mag0, exyz, bxyz, wxyz;
    Vec3d O31 = Vec3d::Zero();
    nm = acc.norm();
    acc0 = nm > 0.1 ? acc / nm : O31;
    nm = mag.norm();
    mag0 = nm > 0.1 ? mag / nm : O31;
    bxyz = Cnb * mag0;
    bxyz[1] = normXY(bxyz);
    bxyz[0] = 0.0;
    wxyz = (Cnb.transpose()) * bxyz;
    exyz = Cnb.row(2).transpose().cross(acc0) + wxyz.cross(mag0);
    exyzInt += exyz * (Ki * ts);
    qnb *= Convert::rv_to_quaternion((gyro - Kp * exyz - exyzInt) * ts);
    Cnb = Convert::quaternion_to_dcm(qnb);
    tk += ts;
}

void AttiAhrs::SetAtti(const Vec3d &init_euler) {
    qnb = Convert::euler_to_quaternion(init_euler);
    Cnb = Convert::quaternion_to_dcm(qnb);
}

void AttiAhrs::Update(const RgioeImuData &imu) {
//    Vec3d gyro = {imu.gyro[1],imu.gyro[0],-imu.gyro[2]};
//    Vec3d acce = {imu.acce[1],imu.acce[0],-imu.acce[2]};
    auto gyro = Vec3d{imu.gyro};
    auto acce = Vec3d{imu.acce};
    Update(gyro,acce);
#if ENABLE_FUSION_RECORDER
    recorder_msg_ahrs_t msg = CREATE_RECORDER_MSG(ahrs);
    msg.timestamp = imu.gpst;
    Vec3d atti = Convert::dcm_to_euler(Cnb);
    msg.data.roll = atti[0] / _deg;
    msg.data.pitch = atti[1]/ _deg;
    msg.data.heading = atti[2]/ _deg;
    Recorder::GetInstance().Record(&msg);
#endif

}

Quad AttiAhrs::GetQnb() const {
    return qnb;
}

Vec3d AttiAhrs::GetAtti() const {
    return Convert::dcm_to_euler(Cnb);
}

void AttiAhrs::SetAtti(const Quad &q) {
    this->qnb = q;
}
