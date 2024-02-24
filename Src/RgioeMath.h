/**
* @file: LooselyCouple2020_cpp extern.h
* @author: rebeater
* @function: 外部依赖库
* @date: 2020/11/11 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
#define LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
//// workaround issue between gcc >= 4.7 and cuda 5.5
//#if (defined __GNUC__) && (__GNUC__ > 4 || __GNUC_MINOR__ >= 7)
//#undef _GLIBCXX_ATOMIC_BUILTINS
//#undef _GLIBCXX_USE_INT128
//#endif

#include<Eigen/Dense>
#include<Eigen/Eigen>
#include "RgioeDefine.h"

#if RGIOE_CONFIG_PRECISE == FP64
#define rgioe_sin sin
#define rgioe_asin asin
#define rgioe_cos cos
#define rgioe_acos acos
#define rgioe_tan tan
#define rgioe_atan2 atan2
#define rgioe_atan atan
#define rgioe_fabs fabs
#define rgioe_sqrt sqrt

typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3d Vec3Hp;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Matrix3d Mat3Hp;
typedef Eigen::Matrix2d Mat2d;

typedef Eigen::Quaternion<RgioeFloatType> Quad;
typedef Eigen::Quaternion<fp64> QuadHp;
#else
#define rgioe_sin sinf
#define rgioe_asin asinf
#define rgioe_cos cosf
#define rgioe_acos acosf
#define rgioe_tan tanf
#define rgioe_atan2 atan2f
#define rgioe_atan atanf
#define rgioe_fabs fabsf
#define rgioe_sqrt sqrtf

typedef Eigen::Vector3f Vec3d;
typedef Eigen::Vector3d Vec3Hp;
typedef Eigen::Vector2f Vec2d;
typedef Eigen::Matrix3f Mat3d;
typedef Eigen::Matrix3d Mat3Hp;
typedef Eigen::Matrix2f Mat2d;
typedef Eigen::Quaternion<RgioeFloatType> Quad;
typedef Eigen::Quaternion<fp64> QuadHp;
#endif
#endif //LOOSELYCOUPLE2020_CPP_MATRIX_LIB_H
