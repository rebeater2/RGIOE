//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H

#include "KalmanFilter.h"
#include "InsCore.h"
#include "StaticDetect.h"
//#define OUTAGE_SUPPORT

extern Option default_option;
extern char CopyRight[];
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
class DataFusion : public KalmanFilter, public Ins, public Singleton<DataFusion> {
 private:
  MatXd Q0;
  Vec3d lb_gnss;
  Vec3d lb_wheel;
  Mat3d Cbv;
  Option opt{};
  uint32_t update_flag;
 /*for RTS */
  std::list<MatXd> matphis;
  std::list<VecXd> Xds;
  std::list<MatXd> matp_pres;
  std::list<MatXd> matp_posts;
  std::list<NavEpoch> navs;

  uint32_t _timeUpdateIdx;/*时间更新计数器*/
 private:
  Mat3Xd _posH() const;
  __attribute__((unused)) Mat3Xd _velH() const;
  IMUSmooth smooth;
  Vec3d _posZ(const Vec3d &pos);
  int _feedBack();
  double p_height_{-INT32_MAX};/*上时刻高程预测*/
  double m_height_{-INT32_MAX};/*保存上时刻高程量测*/

 private:

  __attribute__((unused)) int MeasureNHC();

  /*ZUPT*/
  int MeasureZeroVelocity();

 public:
 protected:
  DataFusion();
  friend Singleton<DataFusion>;
 public:
  uint32_t EpochCounter() const;
  void Initialize(const NavEpoch &ini_nav, const Option &opt);

  int TimeUpdate(const ImuData &imu);

  int MeasureUpdatePos(const Vec3d &pos, const Mat3d &Rk);

  int MeasureUpdatePos(const GnssData &gnssData);

  int MeasureUpdateVel(const Vec3d &vel);
  int MeasureUpdateVel(const double &vel);

  int MeasureUpdateRelativeHeight( double height);

  /**
   *
   * @return 进度: 0开始,1 完成
   */
  bool RtsUpdate();
  NavOutput Output()const;
};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
