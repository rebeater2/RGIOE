/**
* @file rtsim_fusion.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 1/12/22 9:38 PM
* @version 1.0
**/

#include <ros/ros.h>
#include "navplay/include/imu.h"
int main(int argc,char **argv){
  ros::init(argc,argv,"rtsim_fusion_node");
  ros::NodeHandle handle;
  ros::Subscriber imu_sub = handle.subscribe("imu_data",10,)

  ros::spin();
  return 0;
}
