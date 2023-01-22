/**
* @file NavRawReader.h in Project
* @author rebeater
* @comment
* Create on 1/10/22 9:56 AM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVRAWREADER_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVRAWREADER_H_
#include "ReaderBase.h"
#include "RawStruct.h"
using namespace InsCube;
class NavRawReader : public ReaderBase<RawDataDef> {
 public:
  explicit NavRawReader(const std::string &filename);
 public:
  bool ReadNext(RawDataDef &) override;
  double GetTime() const override;
};
#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVRAWREADER_H_
