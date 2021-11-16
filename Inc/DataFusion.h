//
// Created by rebeater on 2020/12/17.
//

#ifndef LOOSELYCOUPLE2020_CPP_DATAFUSION_H
#define LOOSELYCOUPLE2020_CPP_DATAFUSION_H

#include "KalmanFilter.h"
#include "InsCore.h"
#include "StaticDetect.h"
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
  uint32_t _timeUpdateIdx;/*时间更新计数器*/
#if USE_OUTAGE == 1
  Outage otg;
#endif
 private:
  Mat3Xd _posH() const;
  __attribute__((unused)) Mat3Xd _velH() const;
  IMUSmooth smooth;
  Vec3d _posZ(const Vec3d &pos);

  int _feedBack();

 private:
  /*NHC meansurement*/

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

};

#endif //LOOSELYCOUPLE2020_CPP_DATAFUSION_H
