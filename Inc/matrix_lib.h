/**
* @file: LooselyCouple2020_cpp extern.h
* @author: rebeater
* @function: 外部依赖库
* @date: 2020/11/11 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
#define LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
// workaround issue between gcc >= 4.7 and cuda 5.5
#if (defined __GNUC__) && (__GNUC__>4 || __GNUC_MINOR__>=7)
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128
#endif
#include<Eigen/Dense>
#include <Define.h>

typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Matrix2d Mat2d;
typedef Eigen::Quaternion<double> Quad;
#ifndef STATE_CNT
#define STATE_CNT 15
#endif
typedef Eigen::Matrix<double, STATE_CNT, STATE_CNT> MatXd;
typedef Eigen::Matrix<double, STATE_CNT, 1> VecXd;/*列向量*/
typedef Eigen::Matrix<double, 1, STATE_CNT> VecX1d;/*行向量*/
typedef Eigen::Matrix<double, 3, STATE_CNT> Mat3Xd;
typedef Eigen::Matrix<double, 2, STATE_CNT> Mat2Xd;
typedef Eigen::Matrix<double, STATE_CNT, 3> MatX3d;
typedef Eigen::Matrix<double, STATE_CNT, 2> MatX2d;
#endif //LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
