/**
* @file: LooselyCouple2020_cpp wgs84.h.cpp
* @author: rebeater
* @function: TODO
* @date: 2020/11/11 
* @version: 1.0.0
**/
#include "Earth.h"

Vec3d Earth::omega_en_n(const Vec3d &vn, const Vec3Hp &pos) const {
    double h = pos.z(), lat = pos.x();
    auto rm = RM(lat), rn = RN(lat);
    return Vec3d{
            vn.y() / (rn + h), -vn.x() / (rm + h), -vn.y() * tan(lat) / (rn + h)
    };
}

Vec3d Earth::omega_ie_n(double lat) const {
    return Vec3d{omega_e * cos(lat), 0.0, -omega_e * sin(lat)};
}

fp64 Earth::RM(double lat) const {
    auto s = sin(lat);
    return a * (1 - e2) / (pow(1 - e2 * s * s, 1.5));
}

fp64 Earth::RN(double lat) const {
    auto s = sin(lat);
    return a / sqrt(1 - e2 * s * s);
}

RgioeFloatType Earth::dN(double lat1, double lat2, double h) const {
    return (RgioeFloatType)((lat1 - lat2) * (RM(lat1) + h));/*薛总指出的bug*/
}

RgioeFloatType Earth::dN(double lat1, double lat2) const {
    return  (RgioeFloatType)((lat1 - lat2) * RM(lat1));
}

RgioeFloatType Earth::dE(double lon1, double lon2, double lat, double h) const {
    return  (RgioeFloatType)((RN(lat) + h) * cos(lat) * (lon1 - lon2));
}

RgioeFloatType Earth::dE(double lon1, double lon2, double lat) const {
    return  (RgioeFloatType)(RN(lat) * cos(lat) * (lon1 - lon2));
}

Vec3d Earth::distance(double lat1, double lon1, double lat2, double lon2) const {
    auto dn = dN(lat1, lat2);
    auto de = dE(lat1, lon1, lon2);
    return {dn, de, 0};
}

Vec3d Earth::distance(double lat1, double lon1, double lat2, double lon2, double h1, double h2) const {
    auto dn = dN(lat1, lat2, h1 / 2 + h2 / 2);
    auto de = dE(lon1, lon2, lat1 / 2 + lat2 / 2, h1 / 2 + h2 / 2);
    auto dd = h1 - h2;
    return { (RgioeFloatType)dn,  (RgioeFloatType)de, (RgioeFloatType) dd};
}

//deltaPos WGS84::distance(const GnssData &pos1, const GnssData &pos2) const {
//  return distance(pos1.lat * _deg, pos1.lon * _deg, pos2.lat * _deg, pos2.lon * _deg, pos1.height, pos2.height);
//}

void Earth::Update(double lat, double h) {
    double s = sin(lat);
    double s2 = s * s;
    g = g0 * (
            1 + 0.0052790414 * s2 + 0.0000232718 * s2 * s2) + h * (0.0000000043977311 * s2 - 0.0000030876910891)
        + 0.0000000000007211 * h * h;
}

