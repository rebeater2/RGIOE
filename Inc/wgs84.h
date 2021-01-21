/**
* @file: LooselyCouple2020_cpp wgs84.h
* @author: rebeater
* @function: 存放何地球坐标系有关的常量
* @date: 2020/11/10
* @version: 1.0.0
**/
#ifndef LOOSELYCOUPLE2020_CPP_WGS84_H
#define LOOSELYCOUPLE2020_CPP_WGS84_H

#include <nav_struct.h>
#include <matrix_lib.h>

class WGS84 {
public:
    const double omega_e = 7.2921151467e-5; /*地球自转角速度 */
    const double e2 = 0.00669437999013;/*偏心率*/
    const double g0 = 9.7803267715;/*赤道重力加速度*/
    const double GM = 3.986005e14;
    const double mg = 1e-3 * g0;
    const double ug = 1e-6 * g0;
    double a = 6378137.0;/*长轴*/
    Vec3d omega_ie_e = {0,0,omega_e};

     double RM(double lat) ;

     double RN(double lat) ;

     Vec3d omega_en_n(Vec3d vn, Vec3d pos);

     Vec3d omega_ie_n(double lat) ;

     double g(double lat,double h);

    struct delta_d {
        double dn;
        double de;
        double dd;
        double d;
    };

};
extern WGS84 wgs84;
#endif //LOOSELYCOUPLE2020_CPP_WGS84_H
