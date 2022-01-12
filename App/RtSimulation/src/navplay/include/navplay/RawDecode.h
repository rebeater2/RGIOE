/**
* @file RawDecode.h in Project
* @author rebeater
* @comment
* Create on 1/10/22 3:41 PM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_RAWDECODE_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_RAWDECODE_H_
#include "NavTypeDef.h"
void ConvertKyToDouble( const ImuRawKy *raw, ImuData *imu);
#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_RAWDECODE_H_
