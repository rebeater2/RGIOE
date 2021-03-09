//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H

#include <vector>
#include "KalmanFilter.h"
#include "ins_core.h"
class Outage{
private:
    std::vector<int> starts;
    int outage;
    bool flag_enable;
public:
    Outage(int start,int stop,int outage,int step);
    Outage();
    /*tell if gpst is in outage mode */
    bool IsOutage(double gpst);
};
class DataFusion: private KalmanFilter,Ins{
private:
        MatXd Q0;
        Vec3d lb_gnss;
        Vec3d lb_wheel;
        Mat3d Cbv;
        Option opt;
        Outage otg;
private:
    Mat3Xd _pos_h();
    Vec3d _pos_z(Vec3d &pos);
    int _feed_back();

private:
    /*NHC meansurement*/
    int _time_update_idx;
    int MeasureNHC();
    /*ZUPT*/
    int MeasureZeroVelocity();
public:
     DataFusion(NavEpoch &ini_nav,Option &opt);
     int TimeUpdate(ImuData &imu);
     int MeasureUpdatePos(Vec3d &pos,Mat3d &Rk);
     int MeasureUpdatePos(GnssData &gnssData);
     NavOutput Output();
};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
