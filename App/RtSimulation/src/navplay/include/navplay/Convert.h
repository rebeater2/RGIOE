/**
* @file: LooselyCouple2020_cpp convert.h
* @author: rebeater
* @function: 各种类型转
* @date: 2020/11/12 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_CONVERT_H
#define LOOSELYCOUPLE2020_CPP_CONVERT_H

#include <matrix_lib.h>
#include "NavTypeDef.h"

class Convert {
private:
//    const  ;

public:
    static Quad rv_to_quaternion(const Vec3d &rotation_vector);

    static Mat3d rv_to_DCM(const Vec3d &rotation_vector);

    static Vec3d dcm_to_euler(const Mat3d &dcm);

    static Quad euler_to_quaternion(const Vec3d &euler);
    static Mat3d euler_to_dcm(const Vec3d &euler);

    static LatLon qne_to_lla(const Quad &q);
    static Quad lla_to_qne(const LatLon &ll);
    static Mat3d lla_to_cne(const LatLon &ll);
    static Vec3d lla_to_xyz(const Vec3d &lla);

    static Mat3d quaternion_to_dcm(const Quad &q);

    static Vec3d gyro_to_rv(const Vec3d &gyro,const  Vec3d &gyro_pre);

    static Mat3d skew(const Vec3d &v);
};


#endif //LOOSELYCOUPLE2020_CPP_CONVERT_H
