/**
* @file: LooselyCouple2020_cpp convert.cpp
* @author: rebeater
* @function: TODO
* @date: 2020/11/12 
* @version: 1.0.0
**/


#include "Earth.h"
#include "Convert.h"

/**
 * 等效旋转矢量转换为四元数
 * @param rotation_vector
 * @return
 */
Quad Convert::rv_to_quaternion(const Vec3d &rotation_vector) {
    RgioeFloatType mag2 = rotation_vector.x() * rotation_vector.x();
    mag2 += rotation_vector.y() * rotation_vector.y();
    mag2 += rotation_vector.z() * rotation_vector.z();
    if (mag2 < EIGEN_PI * EIGEN_PI) {
        mag2 *= 0.25;
        RgioeFloatType cs = 1.0f - mag2 / 2.0f * (1.0f - mag2 / 12.0f * (1 - mag2 / 30.0f));
        RgioeFloatType sn = 1.0f - mag2 / 6.0f * (1.0f - mag2 / 20.0f * (1 - mag2 / 42.0f));
        return Quad{cs, 0.5f * sn * rotation_vector[0], 0.5f * sn * rotation_vector[1], 0.5f * sn * rotation_vector[2]};
    } else {
        RgioeFloatType mag = rgioe_sqrt(mag2);
        RgioeFloatType s_mag = rgioe_sin(mag / 2);
        Quad q = Quad{
                rgioe_cos(mag / 2),
                rotation_vector[0] * s_mag / mag,
                rotation_vector[1] * s_mag / mag,
                rotation_vector[2] * s_mag / mag,
        };
        if (q.w() < 0)
            q = Quad{
                    -rgioe_cos(mag / 2),
                    -rotation_vector[0] * s_mag / mag,
                    -rotation_vector[1] * s_mag / mag,
                    -rotation_vector[2] * s_mag / mag,
            };
        return q;
    }
}
QuadHp Convert::rv_to_quaternion_hp(const Vec3Hp &rotation_vector) {
    fp64 mag2 = rotation_vector.x() * rotation_vector.x();
    mag2 += rotation_vector.y() * rotation_vector.y();
    mag2 += rotation_vector.z() * rotation_vector.z();
    if (mag2 < EIGEN_PI * EIGEN_PI) {
        mag2 *= 0.25;
        fp64 cs = 1.0f - mag2 / 2.0f * (1.0f - mag2 / 12.0f * (1 - mag2 / 30.0f));
        fp64 sn = 1.0f - mag2 / 6.0f * (1.0f - mag2 / 20.0f * (1 - mag2 / 42.0f));
        return QuadHp{cs, 0.5f * sn * rotation_vector[0], 0.5f * sn * rotation_vector[1], 0.5f * sn * rotation_vector[2]};
    } else {
        fp64 mag = sqrt(mag2);
        fp64 s_mag = sin(mag / 2);
        QuadHp q = QuadHp{
                cos(mag / 2),
                rotation_vector[0] * s_mag / mag,
                rotation_vector[1] * s_mag / mag,
                rotation_vector[2] * s_mag / mag,
        };
        if (q.w() < 0)
            q = QuadHp{
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
    RgioeFloatType norm = rotation_vector[0] * rotation_vector[0];
    norm += rotation_vector[1] * rotation_vector[1];
    norm += rotation_vector[2] * rotation_vector[2];
    double s_qrt = sqrt(norm);
    auto sk = skew(rotation_vector);
    Mat3d eye3 = Eigen::Matrix<RgioeFloatType,3,3>::Identity(3, 3);
    return eye3 + sin(s_qrt) / (s_qrt) * sk + (1 - cos(s_qrt)) / (norm) * sk * sk;
}

Mat3d Convert::quaternion_to_dcm(const Quad &q) {
    return Mat3d(q);
}

LatLon Convert::qne_to_lla(const QuadHp &q) {
    double a = atan(q.y() / q.w());
    double b = atan2(q.z(), q.w());
    double lat = -0.5 * _PI - 2 * a;
    double lon = 2 * b;
    return LatLon{lat, lon};
}

Vec3d Convert::dcm_to_euler(const Mat3d &dcm) {

    RgioeFloatType roll, pitch, heading;
    pitch = rgioe_atan(-dcm(2, 0) / rgioe_sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
    if (dcm(2, 0) <= -0.999) {
        roll = NAN;
        heading = rgioe_atan2(dcm(1, 2) - dcm(0, 1), dcm(0, 2) + dcm(1, 1));
    } else if (dcm(2, 0) >= 0.999) {
        roll = NAN;
        heading = rgioe_atan2(dcm(1, 2) + dcm(0, 1), dcm(0, 2) + dcm(1, 1)) + _PI;
    } else {
        roll = rgioe_atan2(dcm(2, 1), dcm(2, 2));
        heading = rgioe_atan2(dcm(1, 0), dcm(0, 0));
    }
    return Vec3d{roll, pitch, heading};
}

Quad Convert::euler_to_quaternion(const Vec3d &euler) {
    RgioeFloatType roll = euler[0], pitch = euler[1], heading = euler[2];
    RgioeFloatType q0 = rgioe_cos(roll / 2) * rgioe_cos(pitch / 2) * rgioe_cos(heading / 2) + rgioe_sin(roll / 2) * rgioe_sin(pitch / 2) * rgioe_sin(
            heading / 2);
    RgioeFloatType q1 = rgioe_sin(roll / 2) * rgioe_cos(pitch / 2) * rgioe_cos(heading / 2) - rgioe_cos(roll / 2) * rgioe_sin(pitch / 2) * rgioe_sin(
            heading / 2);
    RgioeFloatType q2 = rgioe_cos(roll / 2) * rgioe_sin(pitch / 2) * rgioe_cos(heading / 2) + rgioe_sin(roll / 2) * rgioe_cos(pitch / 2) * rgioe_sin(
            heading / 2);
    RgioeFloatType q3 = rgioe_cos(roll / 2) * rgioe_cos(pitch / 2) * rgioe_sin(heading / 2) - rgioe_sin(roll / 2) * rgioe_sin(pitch / 2) * rgioe_cos(
            heading / 2);
    return Quad{q0, q1, q2, q3};
}


Mat3d Convert::euler_to_dcm(const Vec3d &euler) {
    RgioeFloatType phi = euler[0], theta = euler[1], psi = euler[2];
    Mat3d c;
    c(0, 0) = rgioe_cos(theta) * rgioe_cos(psi);
    c(0, 1) = -rgioe_cos(phi) * rgioe_sin(psi) + rgioe_sin(phi) * rgioe_sin(theta) * rgioe_cos(psi);
    c(0, 2) = rgioe_sin(phi) * rgioe_sin(psi) + rgioe_cos(phi) * rgioe_sin(theta) * rgioe_cos(psi);
    c(1, 0) = rgioe_cos(theta) * rgioe_sin(psi);
    c(1, 1) = rgioe_cos(phi) * rgioe_cos(psi) + rgioe_sin(phi) * rgioe_sin(theta) * rgioe_sin(psi);
    c(1, 2) = -rgioe_sin(phi) * rgioe_cos(psi) + rgioe_cos(phi) * rgioe_sin(theta) * rgioe_sin(psi);
    c(2, 0) = -rgioe_sin(theta);
    c(2, 1) = rgioe_sin(phi) * rgioe_cos(theta);
    c(2, 2) = rgioe_cos(phi) * rgioe_cos(theta);
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
QuadHp Convert::lla_to_qne(const LatLon &ll) {
    fp64 phi = -0.25 * _PI - 0.5 * ll.latitude;
    fp64 lamda = 0.5 * ll.longitude;
    fp64 q0 = cos(phi) * cos(lamda);
    fp64 q1 = -sin(phi) * sin(lamda);
    fp64 q2 = sin(phi) * cos(lamda);
    fp64 q3 = cos(phi) * sin(lamda);
    return QuadHp{q0, q1, q2, q3};
}

Mat3Hp Convert::lla_to_cne(const LatLon &ll) {
    Mat3Hp dcm;
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

Vec3Hp Convert::lla_to_xyz(const Vec3Hp &lla) {
    Vec3Hp re = Vec3Hp::Zero();
    fp64 rn = Earth::Instance().RN(lla[0]);
    re[0] = (rn + lla[2]) * cos(lla[0]) * cos(lla[1]);
    re[1] = (rn + lla[2]) * cos(lla[0]) * sin(lla[1]);
    re[2] = (rn * (1 - Earth::Instance().e2) + lla[2]) * sin(lla[0]);
    return re;
}




