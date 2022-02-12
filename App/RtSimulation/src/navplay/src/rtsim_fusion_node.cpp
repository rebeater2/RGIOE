/**
* @file rtsim_fusion.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment 基于ROS的数据回放仿真程序，利用实际采集的数据进行回放，为了保证和实际结果相同，数据存储是按照实际采集的顺序
 * 确保消息回放结点workr
* Create on 1/12/22 9:38 PM
* @version 1.0
**/

#include <ros/ros.h>
#include "navplay/imu.h"
#include "navplay/gnss.h"
#include "navplay/vel.h"
#include "fmt/format.h"
#include "glog/logging.h"

#include "NavStruct.h"
#include <thread>
#include "boost/circular_buffer.hpp"

boost::circular_buffer<ImuData> imu_queue;
boost::circular_buffer<GnssData> gnss_queue;
boost::circular_buffer<Velocity> vel_data;

#define FUSION_ENGINE GNSS_INS_LOOSELY_COUPLE



void OnImuMsgCallBack(const navplay::imu::ConstPtr &imu_msg) {
  LOG(INFO) << __FUNCTION__ << fmt::format("time:{}, thread_id:", imu_msg.get()->gpst)<<std::this_thread::get_id();
}
void OnGnssMsgCallBack(const navplay::gnss::ConstPtr &gnss_msg) {
  LOG(INFO) << __FUNCTION__ << fmt::format("time:{},thread_id:", gnss_msg.get()->gpst)<<std::this_thread::get_id();
}
void OnVelMsgCallBack(const navplay::vel::ConstPtr &vel_msg) {
  LOG(INFO) << __FUNCTION__ << fmt::format("time:{},thread_id:", vel_msg.get()->gpst)<<std::this_thread::get_id();
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "rtsim_fusion_node");
  ros::NodeHandle handle;
  ros::Subscriber imu_sub = handle.subscribe("imu_data", 100, OnImuMsgCallBack);
  ros::Subscriber gnss_sub = handle.subscribe("gnss_data", 10, OnGnssMsgCallBack);
  ros::Subscriber vel_sub = handle.subscribe("vel_data", 10, OnVelMsgCallBack);
  ros::MultiThreadedSpinner spinner(3);
//  ros::CallbackQueue queue = ros::CallbackQueue::
  spinner.spin();
//  while(ros::ok()){
//    LOG(INFO) <<fmt::format("main time { }",ros::Time::now().toSec());
//    std::this_thread::sleep_for(std::chrono::seconds(1));
//  }
//  ros::spin();
  return 0;
}
