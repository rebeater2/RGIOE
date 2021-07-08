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

extern char CopyRight[];
class DataFusion : public KalmanFilter, public Ins, public Singleton<DataFusion> {
private:
    MatXd Q0;
    Vec3d lb_gnss;
    Vec3d lb_wheel;
    Mat3d Cbv;
    Option opt;
    uint32_t update_flag;
#if USE_OUTAGE == 1
    Outage otg;
#endif
private:
    Mat3Xd _posH()const;
    Mat3Xd _velH() const;

    Vec3d _posZ(const Vec3d &pos);

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
    void Initialize(const NavEpoch &ini_nav,const Option &opt);

    int TimeUpdate(const ImuData &imu);

    int MeasureUpdatePos(const Vec3d &pos,const  Mat3d &Rk);

    int MeasureUpdatePos(const GnssData &gnssData);

    int MeasureUpdateVel(const Vec3d &vel);
    int MeasureUpdateVel(const double &vel);


};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
