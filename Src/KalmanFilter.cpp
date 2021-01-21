/**
 * @file: LooselyCouple2020_cpp KalmanFilter.cpp
 * @author: rebeater
 * @function:  Implement of Kalman Filter
 * @date: 2020/11/28
 * @version: 1.0.0 @note first complement
 *
 **/


#include "KalmanFilter.h"

void KalmanFilter::Predict(MatXd &PHI, MatXd &Q) {
    xd = PHI * xd;
    P = PHI * P * PHI.transpose() + Q;
}

void KalmanFilter::Update(MatXd &H, VecXd &z, MatXd &R) {
    MatXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    xd = xd + K * (z - H * xd);
    P = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H) * P;
}

/**
 * Kalman Update 3 columns
 * @param H
 * @param z
 * @param R
 */
void KalmanFilter::Update(Mat3Xd &H, Vec3d &z, Mat3d &R) {

    MatX3d K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
    xd = xd + K * (z - H * xd);
    MatXd temp = (MatXd::Identity(STATE_CNT, STATE_CNT) - K * H);
    P = temp * P * temp.transpose() + K * R * K.transpose();
}

void KalmanFilter::Reset() {
    /*设置Xd为0*/
    xd.setZero();
}

KalmanFilter::KalmanFilter() {
    xd = VecXd::Zero();
    P = MatXd::Zero();
}
