/**
* @file: LooselyCouple2020_cpp KalmanFilter.h
* @author: rebeater
* @function: a general KalmanFiler
* @date: 2020/11/28 
* @version: 1.0.0
**/


#ifndef LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
#define LOOSELYCOUPLE2020_CPP_KALMANFILTER_H




//#include <utility>

#include "matrix_lib.h"



class KalmanFilter {
    using Vec3x = Eigen::Matrix<double, 1, 1>;
    Vec3x sf;
public:
    VecXd xd;
    MatXd P;
    MatXd Q0;

public:
    void Predict(MatXd &PHI, MatXd &Q);

    void Update(MatXd &H, VecXd &Z, MatXd &R);
    void Update(Mat3Xd &H, Vec3d &Z, Mat3d &R);

    void Reset();

    KalmanFilter(VecXd xd, MatXd P) : xd(std::move(xd)), P(std::move(P)) {}

    KalmanFilter();;
};


#endif //LOOSELYCOUPLE2020_CPP_KALMANFILTER_H
