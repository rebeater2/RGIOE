//
// Created by rebeater on 2020/11/9.
//

#ifndef LOOSELYCOUPLE2020_CPP_INS_CORE_H
#define LOOSELYCOUPLE2020_CPP_INS_CORE_H

#include "nav_struct.h"
#include "matrix_lib.h"
#include "wgs84.h"
#include "convert.h"



/**
 * 维持导航状态的结构体
 */
typedef struct {
    double gpst;/*GPS second*/

    Vec3d pos;/*n-frame position(lat,lon,alt) :d/d/m*/
    Vec3d vn;/*n-frame velocity North East Down :m/a*/
    Vec3d atti;/*attitude forward right down :rad*/

    Vec3d dvn;/*n-frame velocity change :m/a*/
    Vec3d vf_kb;

    Mat3d Cbn;/*Matrix DCM b-frame to e-frame*/
    Mat3d Cne;/*Matrix DCM n-frame to e-frame*/
    Quad Qbn;/*quaternion b-frame to n-frame*/
    Quad Qne;/*quaternion n-frame to e-frame*/

    Vec3d gb;/*gyroscope bias*/
    Vec3d ab;/*accelerator bias*/
    Vec3d gs;/*gyroscope sale factor error*/
    Vec3d as;/*accelerator scale factor error*/

    Vec3d pos_std;
    Vec3d vel_std;
    Vec3d att_std;


} NavEpoch;

std::ostream &operator<<(std::ostream &os, NavEpoch &nav);

NavEpoch makeNavEpoch(double gpst, Vec3d &pos, Vec3d &vn, Vec3d &atti);
NavEpoch makeNavEpoch(NavOutput nav_, Option opt);
/*机械编排主类*/
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
    Vec3d pos_mid, vn_mid;
    double t_pre;
    MatXd phi;

    int _velocity_update(Vec3d &acce, Vec3d &gyro);

    int _position_update();

    int _atti_update(Vec3d &gyro);

    Vec3d _mid_pos();

public:
    Ins(NavEpoch &nav,int d_rate);
    Ins(NavEpoch &nav,ImuData &imu);
//    Ins(NavEpoch &nav,Option &opt);
    int ForwardMechanization(ImuData &imuData);


    MatXd TransferMatrix(ImuPara &para);


    void CompensateIMU(Vec3d &imu, Vec3d &bias, Vec3d &scale) const;
};


#endif //LOOSELYCOUPLE2020_CPP_INS_CORE_H
