/**
* @file: LooselyCouple2020_cpp test_kalman.cpp
* @author: rebeater
* @function: 测试卡尔曼滤波类
* @date: 2020/12/7 
* @version: 1.0.0
**/
#include<iostream>
#include <fstream>

#include "../Src/KalmanFilter.hpp"

using namespace std;

int main() {
    ifstream ifs_zk(R"(/home/rebeater/PycharmProjects/kalman_demo/zk.txt)");
    ofstream ofs_xk(R"(/home/rebeater/PycharmProjects/kalman_demo/xk.txt)");
#define STATE_CNT 2

    KalmanFilter<STATE_CNT,double>::MatXX Q, R, P,H,PHI;
    Q << 0.00984333, 0., 0., 0.0101352;
    cout << "Q=\n" << Q;
    R << 0.00135375, 0., 0., 0.00135375;
    cout<<"R=\n"<<R<<endl;
    PHI =  KalmanFilter<STATE_CNT,double>::MatXX::Identity();
    H = KalmanFilter<STATE_CNT,double>::MatXX::Identity();
    cout<<"PHI=\n"<<PHI<<endl;
    KalmanFilter<STATE_CNT,double>::VecX1 x;
    KalmanFilter<STATE_CNT,double>::MatXX p;
     x << -2.06140251, -3.67284068;
    p.setZero();
    KalmanFilter<STATE_CNT,double> kf{x,p};

    double z[2];
    KalmanFilter<STATE_CNT,double>::VecX1 zk;
    while(!ifs_zk.eof()){
        ifs_zk>>z[0]>>z[1];
        zk[0]=z[0];zk[1]=z[1];
//        cout<<z[0]<<" "<<z[1]<<endl;
        kf.Predict(PHI,Q);
        kf.Update(H,zk,R);
        ofs_xk<<kf.Xd[0]<<" "<<kf.Xd[1]<<endl;
    }
    return 0;
}



