/**
* @file RealtivePos.h in Project
* @author rebeater
* @comment 计算相对位置
* Create on 1/10/22 4:57 PM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_REALTIVEPOS_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_REALTIVEPOS_H_
typedef struct {
  x,
  y,
  z,
};
class RelativePos{
 private:
  double base_lla[3];/*基准站lla*/
 public:
   Update();

};
#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_REALTIVEPOS_H_
