//
// Created by rebeater on 5/24/21.
//

#ifndef LOOSELYCOUPLE2020_CPP_ALIGNMENT_H
#define LOOSELYCOUPLE2020_CPP_ALIGNMENT_H

#include "NavStruct.h"



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
#include "StaticDetect.h"

class AlignBase {

 public:
  NavEpoch nav;
  bool flag_level_finished;
  bool flag_yaw_finished;
 public:
  AlignBase();

  NavOutput getPva() const;

  NavEpoch getNavEpoch() const;

  bool alignFinished() const { return flag_level_finished and flag_yaw_finished; }

  virtual double Update(const GnssData &gnss) { return 0.0; };

  virtual void Update(const ImuData &imu) {};
};

class AlignMoving : public AlignBase {
 private:
  Mat3d Cnb;
  GnssData gnss_pre{};
  IMUSmooth smooth;
  Option option;
 public:
  explicit AlignMoving( const Option &option);

  double Update(const GnssData &gnss) override;

  void Update(const ImuData &imu) override;

  int GnssCheck(const GnssData &gnss);
};

class AlignStatic : AlignBase {
  void Update(const ImuData &imu) override;
};

class AlignDoubleAntenna : AlignBase {
  double Update(const GnssData &gnss) override;

  void Update(const ImuData &imu) override;
};

#endif //LOOSELYCOUPLE2020_CPP_ALIGNMENT_H
