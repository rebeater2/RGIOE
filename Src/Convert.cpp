/**
* @file: LooselyCouple2020_cpp convert.cpp
* @author: rebeater
* @function: TODO
* @date: 2020/11/12 
* @version: 1.0.0
**/


#include <Earth.h>
#include "Convert.h"
/**
 * 等效旋转矢量转换为四元数
 * @param rotation_vector
 * @return
 */
Quad Convert::rv_to_quaternion(const Vec3d &rotation_vector) {
    double mag2 = rotation_vector.x() * rotation_vector.x();
    mag2 += rotation_vector.y() * rotation_vector.y();
    mag2 += rotation_vector.z() * rotation_vector.z();
    if (mag2 < EIGEN_PI * EIGEN_PI) {
        mag2 *= 0.25;
        double cs = 1.0 - mag2 / 2.0 * (1.0 - mag2 / 12.0 * (1 - mag2 / 30.0));
        double sn = 1.0 - mag2 / 6.0 * (1.0 - mag2 / 20.0 * (1 - mag2 / 42.0));
        return Quad{cs, 0.5 * sn * rotation_vector[0], 0.5 * sn * rotation_vector[1], 0.5 * sn * rotation_vector[2]};
    } else {
        double mag = sqrt(mag2);
        double s_mag = sin(mag / 2.);
        Quad q = Quad{
                cos(mag / 2.),
                rotation_vector[0] * s_mag / mag,
                rotation_vector[1] * s_mag / mag,
                rotation_vector[2] * s_mag / mag,
        };
        if (q.w() < 0)
            q = Quad{
                    -cos(mag / 2),
                    -rotation_vector[0] * s_mag / mag,
                    -rotation_vector[1] * s_mag / mag,
                    -rotation_vector[2] * s_mag / mag,
            };
        return q;
    }
}
/**
 * 反对称矩阵
 * @param v [x,y,z]
 * @return matrix 3x3
 * [[0 -z y]
 * [z 0 -x]
 * [-y x 0]]
 */
Mat3d Convert::skew(const Vec3d &v) {
    Mat3d m;
    m.setZero();
    m(0, 1) = -v[2];
    m(0, 2) = v[1];
    m(1, 0) = v[2];
    m(1, 2) = -v[0];
    m(2, 0) = -v[1];
    m(2, 1) = v[0];
    return m;
}

Mat3d Convert::rv_to_DCM(const Vec3d &rotation_vector) {
    double norm = rotation_vector[0] * rotation_vector[0];
    norm += rotation_vector[1] * rotation_vector[1];
    norm += rotation_vector[2] * rotation_vector[2];
    double s_qrt = sqrt(norm);
    auto sk = skew(rotation_vector);
    Mat3d eye3 = Eigen::Matrix3d::Identity(3, 3);
    return eye3 + sin(s_qrt) / (s_qrt) * sk + (1 - cos(s_qrt)) / (norm) * sk * sk;
}

Mat3d Convert::quaternion_to_dcm(const Quad &q) {
    return Mat3d(q);
}

LatLon Convert::qne_to_lla(const Quad &q) {
    double a = atan(q.y() / q.w());
    double b = atan2(q.z(), q.w());
    double lat = -0.5 * _PI - 2 * a;
    double lon = 2 * b;
    return LatLon{lat, lon};
}

Vec3d Convert::dcm_to_euler(const Mat3d &dcm) {

    double roll, pitch, heading;
    pitch = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
    if (dcm(2, 0) <= -0.999) {
        roll = NAN;
        heading = atan2(dcm(1, 2) - dcm(0, 1), dcm(0, 2) + dcm(1, 1));
    } else if (dcm(2, 0) >= 0.999) {
        roll = NAN;
        heading = atan2(dcm(1, 2) + dcm(0, 1), dcm(0, 2) + dcm(1, 1)) + _PI;
    } else {
        roll = atan2(dcm(2, 1), dcm(2, 2));
        heading = atan2(dcm(1, 0), dcm(0, 0));
    }
    return Vec3d{roll, pitch, heading};
}

Quad Convert::euler_to_quaternion(const Vec3d &euler) {
    double roll = euler[0], pitch = euler[1], heading = euler[2];
    double q0 = cos(roll / 2) * cos(pitch / 2) * cos(heading / 2) + sin(roll / 2) * sin(pitch / 2) * sin(
            heading / 2);
    double q1 = sin(roll / 2) * cos(pitch / 2) * cos(heading / 2) - cos(roll / 2) * sin(pitch / 2) * sin(
            heading / 2);
    double q2 = cos(roll / 2) * sin(pitch / 2) * cos(heading / 2) + sin(roll / 2) * cos(pitch / 2) * sin(
            heading / 2);
    double q3 = cos(roll / 2) * cos(pitch / 2) * sin(heading / 2) - sin(roll / 2) * sin(pitch / 2) * cos(
            heading / 2);
    return Quad{q0, q1, q2, q3};
}


Mat3d Convert::euler_to_dcm(const Vec3d &euler) {
    double phi = euler[0], theta = euler[1], psi = euler[2];
    Mat3d c;
    c(0, 0) = cos(theta) * cos(psi);
    c(0, 1) = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
    c(0, 2) = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
    c(1, 0) = cos(theta) * sin(psi);
    c(1, 1) = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
    c(1, 2) = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);
    c(2, 0) = -sin(theta);
    c(2, 1) = sin(phi) * cos(theta);
    c(2, 2) = cos(phi) * cos(theta);
    return c;
}

/**
 * def lla_to_qne(latitude, longitude):
        phi = -0.25 *  pi - 0.5 * latitude
        lamda = 0.5 * longitude
        q0 =  cos(phi) *  cos(lamda)
        q1 = - sin(phi) *  sin(lamda)
        q2 =  sin(phi) *  cos(lamda)
        q3 =  cos(phi) *  sin(lamda)
        return Quaternion(q0, q1, q2, q3)
    */
Quad Convert::lla_to_qne(const LatLon &ll) {
    double phi = -0.25 * _PI - 0.5 * ll.latitude;
    double lamda = 0.5 * ll.longitude;
    double q0 = cos(phi) * cos(lamda);
    double q1 = -sin(phi) * sin(lamda);
    double q2 = sin(phi) * cos(lamda);
    double q3 = cos(phi) * sin(lamda);
    return Quad{q0, q1, q2, q3};
}

Mat3d Convert::lla_to_cne(const LatLon &ll) {
    Mat3d dcm;
    double lat = ll.latitude, lon = ll.longitude;
    dcm(0, 0) = -sin(lat) * cos(lon);
    dcm(0, 1) = -sin(lon);
    dcm(0, 2) = -cos(lat) * cos(lon);
    dcm(1, 0) = -sin(lat) * sin(lon);
    dcm(1, 1) = cos(lon);
    dcm(1, 2) = -cos(lat) * sin(lon);
    dcm(2, 0) = cos(lat);
    dcm(2, 1) = 0.;
    dcm(2, 2) = -sin(lat);
    return dcm;
}

Vec3d Convert::gyro_to_rv(const Vec3d &gyro, const  Vec3d &gyro_pre) {
    return gyro + gyro_pre.cross(gyro) / 12.0;
}

Vec3d Convert::lla_to_xyz(const Eigen::Vector3d &lla) {
    Vec3d re = Vec3d::Zero();
    double rn = Earth::Instance().RN(lla[0]);
    re[0] = (rn + lla[2]) * cos(lla[0]) * cos(lla[1]);
    re[1] = (rn + lla[2]) * cos(lla[0]) * sin(lla[1]);
    re[2] = (rn * (1 - Earth::Instance().e2) + lla[2]) * sin(lla[0]);

//    LOG_FIRST_N(INFO,10)<<std::setprecision(10)<<"lla "<<lla.transpose();
//    LOG_FIRST_N(INFO,10)<<std::setprecision(10)<<"re "<<re.transpose();
    return re;
}
