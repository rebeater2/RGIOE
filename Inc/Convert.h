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
#include "NavStruct.h"

class Convert {
 public:
  /**
   * convert rotation vector to quaternion
   * @param rotation_vector
   * @return Quaternion in terms of rotation vector
   */
  static Quad rv_to_quaternion(const Vec3d &rotation_vector);


  /**
   * convert rotation vector to direction cosine matrix
   * @param rotation_vector
   * @return DCM in terms of rotation vector
   */
  static Mat3d rv_to_DCM(const Vec3d &rotation_vector);


  /**
   * convert direction cosine matrix to euler angle
   * @param dcm DCM
   * @return euler angle in terms of DCM
   */
  static Vec3d dcm_to_euler(const Mat3d &dcm);


  /**
   * convert quaternion to euler angle
   * @param euler :roll pitch heading in NED
   * @return quaternion in terms of euler angle
   */
  static Quad euler_to_quaternion(const Vec3d &euler);


  /**
   * convert euler angle to direction cosine matrix
   * @param euler:roll pitch heading in NED
   * @return DCM in terms of euler angle
   */
  static Mat3d euler_to_dcm(const Vec3d &euler);


  /**
   * convert Qne to latitude and longitude
   * @param q :Qne
   * @return latitude and longitude in rad
   */
  static LatLon qne_to_lla(const Quad &q);


  /**
   * convert latitude and longitude to Qne
   * @param ll latitude and longitude in rad
   * @return Qne
   */
  static Quad lla_to_qne(const LatLon &ll);


  /**
   * convert latitude and longitude to Cne
   * @param ll latitude and longitude in rad
   * @return Cne
   */
  static Mat3d lla_to_cne(const LatLon &ll);


  /**
   * latitude/longitude/height to x/y/z in ECEF
   * @param lla latitude,longitude,height, the units are:rad rad m
   * @return (x,y,z) in ECEF, the unit is m
   */
  static Vec3d lla_to_xyz(const Vec3d &lla);


  /**
   * convert quaternion to DCM
   * @param q
   * @return DCM in terms of quaternion
   */
  static Mat3d quaternion_to_dcm(const Quad &q);

  /**
   * convert gyroscope output to rotation vector, this is in order to compensate the scull error
   * @param gyro gyroscope in increment,the unit is rad
   * @param gyro_pre the previous output of gyroscope
   * @return rotation vector
   */
  static Vec3d gyro_to_rv(const Vec3d &gyro, const Vec3d &gyro_pre);


  /**
   * convert vector to skew matrix, that is <x,y,z> ->
   * [ 0 -z y ]
   * [ z 0 x ]
   * [ y -x 0 ]
   * @param v
   * @return  skew matrix
   */
  static Mat3d skew(const Vec3d &v);
};

#endif //LOOSELYCOUPLE2020_CPP_CONVERT_H
