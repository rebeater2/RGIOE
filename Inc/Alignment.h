//
// Created by rebeater on 5/24/21.
//

#ifndef LOOSELYCOUPLE2020_CPP_ALIGNMENT_H
#define LOOSELYCOUPLE2020_CPP_ALIGNMENT_H

#include "nav_struct.h"



/**
 * AlignBase align;
 * while(!align.isFinished()){
 * align.Update();
 *  }
 *  nav = Align.getNav();
 *
 */
#include "matrix_lib.h"
#include "InsCore.h"
class IMUSmooth {
private:
    ImuData imu_ave;/*均值*/
    ImuData imu_var;
    ImuData imu_pre;/*用于计算短时过0*/
    int up_cnt;
    int static_cnt;
    const int width = 30;/*平均数和std窗口*/
    const double static_std_threshold = 1.5e-5;
    const int static_width = 15;/*连续阈值小于static_std_threshold的判断为静止状态*/
public:
//    IMUSmooth(){};
    IMUSmooth();

    void Update(ImuData &imu);

    ImuData getSmoothedIMU();

    double getStd();

    bool isStatic();
};

class AlignBase {

public:
    NavEpoch nav;
    bool flag_level_finished;
    bool flag_yaw_finished;
public:
    AlignBase() ;

    NavOutput getNav();

    NavEpoch &getNavEpoch();

    bool alignFinished() const { return flag_level_finished and flag_yaw_finished; }

    virtual double Update(GnssData &gnss) { return 0.0; };

    virtual void Update(ImuData &imu) {};
};

class AlignMoving : public AlignBase {
private:
    Mat3d Cnb;
    GnssData gnss_pre{};
    double vel_threshold;
    IMUSmooth smooth;
public:
    explicit AlignMoving(double vel_threshold);

    double Update(GnssData &gnss) override;

    void Update(ImuData &imu) override;
};

/*TODO */
class AlignStatic : AlignBase {
    void Update(ImuData &imu) override;
};

class AlignDoubleAntenna : AlignBase {
    double Update(GnssData &gnss) override;

    void Update(ImuData &imu) override;
};

#endif //LOOSELYCOUPLE2020_CPP_ALIGNMENT_H