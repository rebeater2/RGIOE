//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H


#include "KalmanFilter.h"
#include "InsCore.h"
#include "Singleton.h"
//#define OUTAGE_SUPPORT
#if USE_OUTAGE == 1
#include <vector>
class Outage {
private:
    std::vector<int> starts;
    int outage;
    bool flag_enable;
public:
    Outage(int start, int stop, int outage, int step);

    Outage();

    /*tell if gpst is in outage mode */
    bool IsOutage(double gpst);
};
#endif
class DataFusion : public KalmanFilter, public Ins, public Singleton<DataFusion> {
private:
    MatXd Q0;
    Vec3d lb_gnss;
    Vec3d lb_wheel;
    Mat3d Cbv;
    Option opt;
#if USE_OUTAGE == 1
    Outage otg;
#endif
private:
    Mat3Xd _posH();

    Vec3d _posZ(Vec3d &pos);

    int _feedBack();

private:
    /*NHC meansurement*/
    int _time_update_idx;

    int MeasureNHC();

    /*ZUPT*/
    int MeasureZeroVelocity();

public:
protected:
    DataFusion();
    friend Singleton<DataFusion>;
public:
    void Initialize(NavEpoch &ini_nav, Option &opt);

    int TimeUpdate(ImuData &imu);

    int MeasureUpdatePos(Vec3d &pos, Mat3d &Rk);

    int MeasureUpdatePos(GnssData &gnssData);


};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
