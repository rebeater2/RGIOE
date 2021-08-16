/**
* @file: LooselyCouple2020_cpp wgs84.h.cpp
* @author: rebeater
* @function: TODO
* @date: 2020/11/11 
* @version: 1.0.0
**/
#include <WGS84.h>

Vec3d WGS84::omega_en_n(Vec3d vn, Vec3d pos) {
    double h = pos.z(), lat = pos.x();
    auto rm = RM(lat), rn = RN(lat);
    return Vec3d{
            vn.y() / (rn + h), -vn.x() / (rm + h), -vn.y() * tan(lat) / (rn + h)
    };
}

Vec3d WGS84::omega_ie_n(double lat) {
    return Vec3d{omega_e * cos(lat), 0.0, -omega_e * sin(lat)};
}

double WGS84::RM(double lat) const {
    auto s = sin(lat);
    return a * (1 - e2) / (pow(1 - e2 * s * s, 1.5));
}

double WGS84::RN(double lat) const {
    auto s = sin(lat);
    return a / sqrt(1 - e2 * s * s);
}

double WGS84::dN(double lat1, double lat2) const {
    return (lat1 - lat2) * RM(lat1);/*todo 或许应该用(lat1+lat2)/2*/
}

double WGS84::dE(double lat, double lon1, double lon2) const {
    return RN(lat) * cos(lat) * (lon1 - lon2);
}

deltaPos WGS84::distance(double lat1, double lon1, double lat2, double lon2) const {
    auto dn = dN(lat1, lat2);
    auto de = dE(lat1, lon1, lon2);
    return {dn, de, 0, sqrt(dn * dn + de * de)};
};

deltaPos WGS84::distance(double lat1, double lon1, double lat2, double lon2, double h1, double h2) const {
    auto dn = dN(lat1, lat2);
    auto de = dE(lat1, lon1, lon2);
    auto dd = h1 - h2;
    return {dn, de, dd, sqrt(dn * dn + de * de + dd * dd)};
};

deltaPos WGS84::distance(GnssData &pos1, GnssData &pos2) const {
    return distance(pos1.lat * _deg, pos1.lon * _deg, pos2.lat * _deg, pos2.lon * _deg,pos1.height,pos2.height);
}

void WGS84::Update(double lat, double h) {
    double s = sin(lat);
    double s2 = s * s;
    g = g0 * (
            1 + 0.0052790414 * s2 + 0.0000232718 * s2 * s2) + h * (
            0.0000000043977311 * s2 - 0.0000030876910891
    ) + 0.0000000000007211 * h * h;
}

WGS84 wgs84;