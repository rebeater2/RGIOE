/******************************************************************************
 * RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter
 * Copyright (C) 2024                                                         *
 * Author : rebeater                                                          *
 * Contact : rebeater@qq.com                                                  *
 ******************************************************************************/

/**
* @file AttiAhrs.h in RGIOE
* @author linfe
* @comment 
* Create on 3/17/2024
* @version 1.0
**/

#ifndef RGIOE_ATTIAHRS_H
#define RGIOE_ATTIAHRS_H

#include "RgioeMath.h"
#include "RgioeDataType.h"

enum AttiAhrsMethod_t {
    AttiAhrsMethod_Mahony,
    AttiAhrsMethod_EKF,
};

class AttiAhrs {
public:
    AttiAhrs(AttiAhrsMethod_t method, RgioeFloatType ts);

    ~AttiAhrs() = default;

public:
    void SetAtti(const Vec3d &init_euler);
    void SetAtti(const Quad &qnb);
    void Update(const Vec3d &acce, const Vec3d &gyro);
    void Update(const RgioeImuData &imu);
    void Update(const Vec3d &acce, const Vec3d &gyro, const Vec3d &mag);
    Quad GetQnb()const;
    Vec3d GetAtti()const;

private:
    Quad qnb;
    Mat3d Cnb;
    AttiAhrsMethod_t method;
    RgioeFloatType ts;
    RgioeFloatType Kp,Ki,tk;
    Vec3d exyzInt, ebMax;
};

#endif //RGIOE_ATTIAHRS_H
