/**
* @file: LooselyCouple2020_cpp wgs84.h.cpp
* @author: rebeater
* @function: TODO
* @date: 2020/11/11 
* @version: 1.0.0
**/
#include <wgs84.h>

  Vec3d WGS84::omega_en_n(Vec3d vn,  Vec3d pos) {
    double h = pos.z(),lat = pos.x();
    auto rm = RM(lat), rn = RN(lat);
    return Vec3d{
            vn.y() / (rn + h), -vn.x() / (rm + h), -vn.y() * tan(lat) / (rn + h)
    };
}

 Vec3d WGS84::omega_ie_n(double lat)  {
    return Vec3d{omega_e * cos(lat), 0.0, -omega_e * sin(lat)};
}

 double WGS84::RM(double lat)  {
    auto s = sin(lat);
    return a * (1 - e2) / (pow(1 - e2 * s * s, 1.5));
}

 double WGS84::RN(double lat)  {
    auto s = sin(lat);
    return a / sqrt(1 - e2 * s * s);
}
 double WGS84::g(double lat,double h){
    double s = sin(lat);
    double s2 = s*s;
    auto ge = g0 * (
            1 + 0.0052790414 * s2 + 0.0000232718 * s2 *s2) + h * (
            0.0000000043977311 * s2 - 0.0000030876910891
    ) + 0.0000000000007211 * h *h;
    return ge;
}
WGS84 wgs84;