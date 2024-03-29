/**
* @file: LooselyCouple2020_cpp wgs84.h
* @author: rebeater
* @function: 存放何地球坐标系有关的常量
* @date: 2020/11/10
* @version: 1.0.0
**/
#ifndef LOOSELYCOUPLE2020_CPP_WGS84_H
#define LOOSELYCOUPLE2020_CPP_WGS84_H

//#include "NavStruct.h"
#include "RgioeMath.h"

struct deltaPos {
    double dn;
    double de;
    double dd;
    double d;
};

class Earth {
private:
    Earth() = default;

    ~Earth() = default;

    Earth(const Earth &rhs) = default;

    Earth &operator=(const Earth &rhs) { return *this; }

public:
    static Earth &Instance() {
        static Earth wgs;
        return wgs;
    }

public:
    const RgioeFloatType omega_e = 7.2921151467e-5;                /*地球自转角速度 */
    const RgioeFloatType e2 = 0.00669437999013;                    /*偏心率*/
    const RgioeFloatType g0 = 9.7803267715;                        /*赤道重力加速度*/
    const RgioeFloatType GM = 3.986005e14;                        /*万有引力常量GM=gR^2*/
    const RgioeFloatType mg = 1e-3 * g0;                            /*1e-3 倍重力*/
    const RgioeFloatType ug = 1e-6 * g0;                            /*ug*/
    const RgioeFloatType a = 6378137.0;                            /*半长轴*/
    RgioeFloatType g = g0;                                        /*重力*/
    Vec3d omega_ie_e = {0, 0, (RgioeFloatType) omega_e};

    fp64 RM(double lat) const;

    fp64 RN(double lat) const;

    Vec3d omega_en_n(const Vec3d &vn, const Vec3Hp &pos) const;

    Vec3d omega_ie_n(double lat) const;

    void Update(double lat, double h);

    RgioeFloatType dN(double lat1, double lat2) const;

    RgioeFloatType dN(double lat1, double lat2, double h1) const;

    RgioeFloatType dE(double lon1, double lon2, double lat) const;

    RgioeFloatType dE(double lon1, double lon2, double lat, double h) const;

    /**
     * 地球表面两点之间距离
     * @param lat1 单位 rad
     * @param lon1 单位 rad
     * @param lat2 单位 rad
     * @param lon2 单位 rad
     * @return 北东地三轴向量和模长
     */
    Vec3d distance(double lat1, double lon1, double lat2, double lon2) const;

    /**
     * 地球表面两点之间距离
     * @param lat1
     * @param lon1
     * @param lat2
     * @param lon2
     * @param h1 单位 m
     * @param h2 单位 m
     * @return m
     */
    Vec3d distance(double lat1, double lon1, double lat2, double lon2, double h1, double h2) const;

//  deltaPos distance(const GnssData &pos1, const GnssData &pos2) const;
/*
    delta_d distance(NavEpoch nav1, NavEpoch nav2) const;*/

};

#endif //LOOSELYCOUPLE2020_CPP_WGS84_H
