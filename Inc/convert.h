/**
* @file: LooselyCouple2020_cpp convert.h
* @author: rebeater
* @function: TODO
* @date: 2020/11/12 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_CONVERT_H
#define LOOSELYCOUPLE2020_CPP_CONVERT_H

#include <matrix_lib.h>
#include <nav_struct.h>

class convert {
private:
//    const  ;

public:
    static Quad rv_to_quaternion(Vec3d &rotation_vector);

    static Mat3d rv_to_DCM(Vec3d &rotation_vector);

    static Vec3d dcm_to_euler(Mat3d &dcm);

    static Quad euler_to_quaternion(Vec3d &euler);
    static Mat3d euler_to_dcm(Vec3d &euler);

    static LatLon qne_to_lla(Quad &q);
    static Quad lla_to_qne(LatLon &ll);
    static Mat3d lla_to_cne(LatLon &ll);
    static Vec3d lla_to_xyz(Vec3d &lla);

    static Mat3d quaternion_to_dcm(Quad &q);

    static Vec3d gyro_to_rv(Vec3d &gyro, Vec3d &gyro_pre);

    static Mat3d skew(const Vec3d &v);
};


#endif //LOOSELYCOUPLE2020_CPP_CONVERT_H
