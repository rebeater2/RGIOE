//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#include "KalmanFilter.h"
#include "ins_core.h"
class DataFusion: private KalmanFilter,Ins{
private:
        MatXd Q0;
        Vec3d lb_gnss;
        Option opt;
private:
    Mat3Xd _pos_h();
    Vec3d _pos_z(Vec3d &pos);
    int _feed_back();
public:
     DataFusion(NavEpoch &ini_nav,Option &opt);
     int TimeUpdate(ImuData &imu);
     int MeasureUpdatePos(Vec3d &pos,Mat3d &Rk);
     int MeasureUpdatePos(GnssData &gnssData);
     NavOutput Output();
};
#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
