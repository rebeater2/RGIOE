//
// Created by rebeater on 5/24/21.
//

#ifndef LOOSELYCOUPLE2020_CPP_ALIGNMENT_H
#define LOOSELYCOUPLE2020_CPP_ALIGNMENT_H

#include "RgioeDataType.h"



/**
 * AlignBase align;
 * while(!align.isFinished()){
 * align.Update();
 *  }
 *  nav = Align.getNav();
 *
 */
#include "RgioeMath.h"
#include "InsCore.h"
#include "StaticDetect.h"
#include "AttiAhrs.h"

class AlignBase {
public:
    AlignBase();

    NavOutput getPva() const;

    const NavEpoch &getNavEpoch() const;

    virtual const bool alignFinished() const { return flag_level_finished and flag_yaw_finished; }

    virtual double Update(const RgioeGnssData &gnss) { return 0.0; };

    virtual void Update(const RgioeImuData &imu) {};
public:
    NavEpoch nav;
    bool flag_level_finished;
    bool flag_yaw_finished;
};

class AlignMoving {
public:

    explicit AlignMoving(const RgioeOption &RgioeOption);

    double Update(const RgioeGnssData &gnss);

    void Update(const RgioeImuData &imu);

    int GnssCheck(const RgioeGnssData &gnss);

    int SetOption(const RgioeOption &RgioeOption);

    const bool alignFinished() const;

    bool levelFinished() const;

public:
    NavEpoch nav;
    bool flag_level_finished;
    bool flag_yaw_finished;
private:
    RgioeGnssData gnss_pre{};
    IMUSmooth smooth;
    RgioeOption option;
    Vec3d vn_pre;
    Vec3d acc_pre;
    double gpst_pre{};
    AttiAhrs ahrs;

};

class AlignStatic : public AlignBase {
public:
    void Update(const RgioeImuData &imu) override;
};

class AlignDoubleAntenna : AlignBase {
    double Update(const RgioeGnssData &gnss) override;

    void Update(const RgioeImuData &imu) override;
};

#endif //LOOSELYCOUPLE2020_CPP_ALIGNMENT_H
