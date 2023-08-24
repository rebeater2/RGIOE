//
// Created by rebeater on 2020/11/9.
//

#ifndef LOOSELYCOUPLE2020_CPP_INSCORE_H
#define LOOSELYCOUPLE2020_CPP_INSCORE_H

#include "RgioeDefine.h"
#include "RgioeDataType.h"
#include "RgioeMath.h"
#include "Earth.h"
#include "Convert.h"

/**
 * 维持导航状态的结构体
 */
typedef struct {
    double gpst;/*GPS second*/

    Vec3Hp pos;/*n-frame position(lat,lon,alt) :d/d/m*/
    Vec3d vn;/*n-frame velocity North East Down :m/a*/
    Vec3d atti;/*attitude forward right down :rad*/

    Vec3d dvn;/*n-frame velocity change :m/a*/
    Vec3d vf_kb;

    Mat3d Cbn;/*Matrix DCM b-frame to e-frame*/
    Mat3Hp Cne;/*Matrix DCM n-frame to e-frame*/
    Quad Qbn;/*quaternion b-frame to n-frame*/
    QuadHp Qne;/*quaternion n-frame to e-frame*/

    Vec3d gb;/*gyroscope bias*/
    Vec3d ab;/*accelerator bias*/
    Vec3d gs;/*gyroscope sale factor error*/
    Vec3d as;/*accelerator scale factor error*/

    Vec3d pos_std;
    Vec3d vel_std;
    Vec3d att_std;
    double kd;/*里程计比例因子*/
    NavInfo info;
    int week;

} NavEpoch;


NavEpoch makeNavEpoch(double gpst, Vec3Hp &pos, Vec3d &vn, Vec3d &atti);

NavEpoch makeNavEpoch(NavOutput nav_, RgioeOption opt);

/* 机械编排主类 */
class Ins {
public:
    NavEpoch nav;
    double dt;
    Mat3d eye3;
public:
    Vec3d _acce_pre;
    Vec3d _gyro_pre;
    Vec3d omega_en_n;
    Vec3d omega_ie_n;
private:
    Vec3d vn_mid;
    Vec3Hp pos_mid;


    int _velocity_update(const Vec3d &acce, const Vec3d &gyro);

    int _position_update();

    int _atti_update(const Vec3d &gyro);

    void _extrapolate();

public:
    Ins();

    ~Ins();

public:
    void InitializePva(const NavEpoch &nav, int d_rate);

    void InitializePva(const NavEpoch &nav, const RgioeImuData &imu);

//    Ins(NavEpoch &nav,RgioeOption &opt);
    int ForwardMechanization(const RgioeImuData &imuData);

    int ForwardMechanization(const Vec3d &acce, const Vec3d &gyro);


    Vec3d CompensateIMU(const Vec3d &imu, const Vec3d &bias, const Vec3d &scale) const;

    virtual NavOutput Output() const;
};

#endif //LOOSELYCOUPLE2020_CPP_INSCORE_H

