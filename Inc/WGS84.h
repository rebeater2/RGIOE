/**
* @file: LooselyCouple2020_cpp wgs84.h
* @author: rebeater
* @function: 存放何地球坐标系有关的常量
* @date: 2020/11/10
* @version: 1.0.0
**/
#ifndef LOOSELYCOUPLE2020_CPP_WGS84_H
#define LOOSELYCOUPLE2020_CPP_WGS84_H

#include "NavStruct.h"
#include <matrix_lib.h>

struct deltaPos {
  double dn;
  double de;
  double dd;
  double d;
};

class WGS84 {
 private:
  WGS84() = default;
  ~WGS84() = default;
  WGS84(const WGS84 &rhs) = default;
  WGS84 &operator=(const WGS84 &rhs) {return *this;}
 public:
  static WGS84 &Instance() {
	static WGS84 wgs;
	return wgs;
  }
 public:
  const double omega_e = 7.2921151467e-5; /*地球自转角速度 */
  const double e2 = 0.00669437999013;/*偏心率*/
  const double g0 = 9.7803267715;/*赤道重力加速度*/
  const double GM = 3.986005e14;
  const double mg = 1e-3 * g0;
  const double ug = 1e-6 * g0;
  double a = 6378137.0;/*长轴*/
  double g = g0;
  Vec3d omega_ie_e = {0, 0, omega_e};

  double RM(double lat) const;

  double RN(double lat) const;

  Vec3d omega_en_n(Vec3d vn, Vec3d pos) const;

  Vec3d omega_ie_n(double lat) const;

  void Update(double lat, double h);

  double dN(double lat1, double lat2) const;
  double dN(double lat1, double lat2,double h1) const;

  double dE(double lon1, double lon2,double lat) const;
  double dE( double lon1, double lon2,double lat,double h) const;

  deltaPos distance(double lat1, double lon1, double lat2, double lon2) const;

  deltaPos distance(double lat1, double lon1, double lat2, double lon2, double h1, double h2) const;

  deltaPos distance(const GnssData &pos1, const GnssData &pos2) const;
/*
    delta_d distance(NavEpoch nav1, NavEpoch nav2) const;*/

};
#endif //LOOSELYCOUPLE2020_CPP_WGS84_H
