//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H

#include "KalmanFilter.hpp"
#include "InsCore.h"
#include "StaticDetect.h"
#include "DcmEstimator.h"
#include "MacroLib.h"


#if ENABLE_FUSION_RECORDER
#include "Recorder/Recorder.h"
#endif


extern RgioeOption default_option;

template<typename T>
class Singleton {
public:
    static T &Instance() {
        static T s_Instance;
        return s_Instance;
    }

protected:
    Singleton() = default;

    ~Singleton() = default;

private:
    Singleton(const Singleton &rhs) = default;

    Singleton &operator=(const Singleton &rhs) {}
};

#define USE_MATRIX_WITH_KALMAN_DIM(type) using type = KalmanFilter<STATE_CNT, RgioeFloatType>::type

class DataFusion : public Ins {
    class Monitor{
    public:
        void OnTimeUpdate();
        void OnMeasUpdate();
        void OnReject();
        uint32_t GetMaxIntegrateIter() const;
        uint32_t GetRejectCnt() const;
        uint32_t GetEkfCnt() const;
        uint32_t GetOutageCnt() const;
    private:
        uint32_t reject_cnt{0};
        uint32_t no_update_cnt{0};
        uint32_t meas_update_cnt{0};
        uint32_t time_update_cnt{0};
        uint32_t no_update_cnt_min{UINT32_MAX};
        uint32_t no_update_cnt_max{0};
    };
public:
    USE_MATRIX_WITH_KALMAN_DIM(MatXX);
    USE_MATRIX_WITH_KALMAN_DIM(Mat3X);
    USE_MATRIX_WITH_KALMAN_DIM(Mat2X);
    USE_MATRIX_WITH_KALMAN_DIM(MatX3);
    USE_MATRIX_WITH_KALMAN_DIM(VecX1);
    USE_MATRIX_WITH_KALMAN_DIM(Vec1X);
public:
    DataFusion();

public:
    /**
     * the number of epoches
     * @return
     */
    uint32_t EpochCounter() const;

    /**
     * initial the datafusion class
     * @param ini_nav initial state
     * @param opt RgioeOption for the algorithm
     */
    void Initialize(const NavEpoch &ini_nav, const RgioeOption &opt);

    /**
     * time update of extend kalman filter
     * @param imu : IMU data
     * @return 0
     */
    int TimeUpdate(const RgioeImuData &imu);

    /**
     * position update for extend kalman filter
     * @param pos position in (rad rad m)
     * @param Rk error matrix of the position
     * @return 0
     */
    int MeasureUpdatePos(const Vec3Hp &pos, const Mat3d &Rk);

    /**
     * GNSS position update for extend kalman filter.
     * the GNSS data must include coordinate and pos_std,
     * if mode is not available,reimplement the function named "GNSScheck"
     * @param gnssData
     * @return 0 for OK,1 for discarded GNSS data
     */
    int MeasureUpdatePos(const RgioeGnssData &gnssData);

    /**
     * Velocity update, NHC or odometer measurement
     * @param vel <x,y,z> in body frame
     * @return 0
     */
    int MeasureUpdateVel(const Vec3d &vel);

    /**
     * forward velocity update
     * @param vel: forward velocity
     * @return 0
     */
    int MeasureUpdateVel(const fp32 &vel);

    /**
     * pressure height update
     * @param height
     * @return
     */
    float MeasureUpdateRelativeHeight(double height);

#if RGIOE_ENABLE_RTS == 1

    /**
     * RTS 反向平滑
     * @return 进度: 0开始,1 完成
     */
    bool RtsUpdate();
#endif

    /**
     * output position,velocity and attitude
     * @return NavOutput
     */
    NavOutput Output() const override;

private:
    /**
   * Zero velocity update,ZUPT and ZIHR implement
   * @return 0
   */
    int MeasureZeroVelocity();

    /** @brief NHC update
     * @return 0
     */
    int MeasureNHC();

    MatXX TransferMatrix(const ImuPara &para);

public:
    DcmEstimator estimator_{};            /*DCM estimator for install angle */
public:
    /*base private*/
    KalmanFilter<STATE_CNT, RgioeFloatType> kf;
    MatXX Q0;                            /*Matrix for Q*/
    Vec3d lb_gnss;                        /*Gnss level arm in meter*/
    Vec3d lb_wheel;                    /*wheel level arm in meter*/
    Mat3d Cbv;                            /*Cbv,DCM from body frame to vehicle frame*/
    RgioeOption opt{};                         /*global RgioeOption for data fusion*/
    uint32_t update_flag;                    /*flag,set to 1 when measurement is coming*/
    VecX1 kfXd;                             /*state vector for kalman filter*/
    int32_t update_iter;
    NavEpoch first_epoch;
/*#if REAL_TIME_MODE != 1
    *//*for RTS *//*

    std::list<MatXX> matphis;             *//* save mat PHI*//*
    std::list<Vec1X> Xds;                 *//*save vector xd*//*
    std::list<MatXX> matp_pres;            *//*save predicted mat P*//*
    std::list<MatXX> matp_posts;            *//*save updated mat P*//*
    std::list<NavEpoch> navs;                *//*save status*//*
#endif*/
    uint32_t _timeUpdateIdx;                /*number of time updates*/
private:
    KalmanFilter<STATE_CNT, RgioeFloatType>::Mat3X _posH() const;                    /* mat H for position update*/
    Mat3X _velH() const;    /* mat H for velocity update*/
    IMUSmooth smooth;    /*Static detector*/
    Vec3d _posZ(const Vec3Hp &pos);        /* calculate delta Z*/
    int _feedBack(const KalmanFilter<STATE_CNT,RgioeFloatType>::Vec1X &kd);                     /*feedback for position,velocity and height*/
    Monitor monitor;
private:
    /*for height update*/
    double p_height{INT32_MIN};         /*上时刻高程预测*/
    double gnss_height{INT32_MIN};        /*保存上时刻高程量测*/
    int base_height_is_set = 0;            /* set to 1 when GNSS is comming*/

};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
