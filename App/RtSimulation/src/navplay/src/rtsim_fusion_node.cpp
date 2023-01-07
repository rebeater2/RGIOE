/**
* @file rtsim_fusion.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment 基于ROS的数据回放仿真程序，利用实际采集的数据进行回放，为了保证和实际结果相同，数据存储是按照实际采集的顺序
 * 确保消息回放结点workr
* Create on 1/12/22 9:38 PM
* @version 1.0
**/

#include "LooselyCouple.h"
#include "FileIO.h"

#include <ros/ros.h>
#include "navplay/imu.h"
#include "navplay/gnss.h"
#include "navplay/vel.h"
#include "navplay/debug.h"
#include "fmt/format.h"
#include "glog/logging.h"

#include "RgioeDataType.h"
#include <thread>
#include <nav_msgs/Path.h>
#include <Earth.h>
#include "Convert.h"
#include "boost/circular_buffer.hpp"

boost::circular_buffer<ImuData> imu_queue;
boost::circular_buffer<GnssData> gnss_queue;
boost::circular_buffer<Velocity> vel_queue;

#define FUSION_ENGINE RGIOE

/*初始经度、纬度、高程*/
double base_lla[3];

/**
 * @brief Check whether gnss data is valid
 * @param gnss: GNSS data
 * @return true for valid
 */
bool GnssCheck(const GnssData &gnss) {
  if (gnss.gpst <= 0) {
	return false;
  }
  if (gnss.mode == SPP) {
	return true;
  }
  if (gnss.mode == RTK_DGPS) {
	return true;
  } else if (gnss.mode == RTK_FLOAT || gnss.mode == RTK_FIX) {
	return true;
  }
  return false;
};
/**
 * output GNSS message
 * @param os
 * @param gnss
 * @return
 *//*
ostream &operator<<(ostream &os, const GnssData &gnss) {
  os << fmt::format("{:.3f} {:.8f} {:.8f}  {:.3f}  {:.3f} {:.3f} {:.3f} {:d} {:d}",
					gnss.gpst,
					gnss.lat,
					gnss.lon,
					gnss.height,
					gnss.pos_std[0],
					gnss.pos_std[1],
					gnss.pos_std[2],
					gnss.ns,
					gnss.mode
  );
  return os;
}*/

/**
 * @brief
 * @param imu_msg
 */
void OnImuMsgCallBack(const navplay::imu::ConstPtr &imu_msg) {
//  LOG_FIRST_N(INFO, 100) << __FUNCTION__ << fmt::format("time:{}, thread_id:", imu_msg.get()->gpst)
//						 << std::this_thread::get_id();
  ImuData imu;
  imu.gpst = imu_msg->gpst - 1;
  for (int i = 0; i < 3; i++) {
	imu.acce[i] = (double)(imu_msg->acce[i]);
	imu.gyro[i] = (double)(imu_msg->gyro[i]);
  }
  imu_queue.push_back(imu);
}

void OnGnssMsgCallBack(const navplay::gnss::ConstPtr &gnss_msg) {
  GnssData gnss;
  gnss.gpst = gnss_msg->gpst;
  gnss.lat = gnss_msg->lat;
  gnss.lon = gnss_msg->lon;
  gnss.height = gnss_msg->h;
  gnss.mode = gnss_msg->mode;
  gnss.ns = gnss_msg->ns;
  gnss.yaw = -1;
  for (int i = 0; i < 3; i++) {
	gnss.pos_std[i] = (float)(gnss_msg->pos_std[i]);
  }
  gnss_queue.push_back(gnss);
}
void OnVelMsgCallBack(const navplay::vel::ConstPtr &vel_msg) {
  LOG_FIRST_N(INFO, 1) << __FUNCTION__ << fmt::format("time:{},thread_id:", vel_msg.get()->gpst)
					   << std::this_thread::get_id();
  Velocity vel;
  vel.gpst = vel_msg->gpst;
  vel.angular = vel_msg->angular;
  vel.forward = vel_msg->forward;
  vel_queue.push_back(vel);
}
static nav_msgs::Path path{};
void publishResult(ros::Publisher &pub, const NavOutput &nav) {
  LOG_EVERY_N(INFO, 125) << fmt::format("output:{} {} {} {}", nav.gpst, nav.lat, nav.lon, nav.height);
  geometry_msgs::PoseStamped pose_stamped;
  auto is_int = [](double a) { return fabs(a - int(a)) < 0.01; };
  if (!is_int(nav.gpst)) {
	pub.publish(path);
	return;
  }
  auto d = WGS84::Instance().distance(nav.lat * _deg,
									  nav.lon * _deg,
									  base_lla[0] * _deg,
									  base_lla[1] * _deg,
									  nav.height,
									  base_lla[2]);
  pose_stamped.pose.position.x = d.de;
  pose_stamped.pose.position.y = d.dn;
  pose_stamped.pose.position.z = -d.dd;
  Quad quad =
	  Convert::euler_to_quaternion({nav.atti[0] * _deg, nav.atti[1] * _deg, nav.atti[2] * _deg});
  //	auto quad = tf::createQuaternionFromRPY(nav.atti[0] * _deg, nav.atti[1] * _deg, nav.atti[2] * _deg);
  pose_stamped.pose.orientation.x = quad.x();
  pose_stamped.pose.orientation.y = quad.y();
  pose_stamped.pose.orientation.z = quad.z();
  pose_stamped.pose.orientation.w = quad.w();
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "ins";
  path.poses.push_back(pose_stamped);
  path.poses.size() > 10000 ? path.poses.clear() : void(0);
  pub.publish(path);
}

void publishDebugInfo(ros::Publisher &pub, const NavOutput &nav) {
  navplay::debug debug;
  debug.gpst = nav.gpst;
  for (int i = 0; i < 3; i++) {
	debug.pos_std[i] = nav.pos_std[i];
	debug.atti_std[i] = nav.atti_std[i];
	debug.vel_std[i] = nav.atti_std[i];
	debug.gb[i] = nav.gb[i];
	debug.ab[i] = nav.ab[i];
  }
  pub.publish(debug);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging("log");
  google::LogToStderr();
  imu_queue.set_capacity(10);
  gnss_queue.set_capacity(1);
  vel_queue.set_capacity(10);
  ros::init(argc, argv, "rtsim_fusion_node");
  ros::NodeHandle handle;
  bool save_result;
  std::string result_path;
  handle.param<double>("base_lat", base_lla[0], 0);
  handle.param<double>("base_lon", base_lla[1], 0);
  handle.param<double>("base_height", base_lla[2], 0);
  handle.param<bool>("save_result", save_result, false);
  handle.param<std::string>("result_path", result_path, "./result.rtnav");
  LOG(INFO) << fmt::format("Base station:", base_lla[0], base_lla[1], base_lla[2]);
  ros::Subscriber imu_sub = handle.subscribe("imu_data", 10, OnImuMsgCallBack);
  ros::Subscriber gnss_sub = handle.subscribe("gnss_data", 10, OnGnssMsgCallBack);
  ros::Subscriber vel_sub = handle.subscribe("vel_data", 10, OnVelMsgCallBack);
  ros::Publisher rst_pub = handle.advertise<nav_msgs::Path>("PVA", 10);
  ros::Publisher debug_pub = handle.advertise<navplay::debug>("debug_info", 10);

  /*multi thread*/
  ros::AsyncSpinner s(3);
  s.start();
  Option opt = default_option;
  opt.align_mode = ALIGN_MOVING;
  opt.odo_scale = 0.976;
  opt.odo_std = 0.01;
  opt.nhc_enable = true;
  opt.nhc_std[0] = 0.01;
  opt.nhc_std[1] = 0.01;
  opt.angle_bv[0] = 0;
  opt.angle_bv[1] = -0.8 * _deg;
  opt.angle_bv[2] = 2.15 * _deg;
  opt.odo_enable = 0;
  opt.nhc_enable = 0;
  opt.align_vel_threshold = 0.5;
  opt.zupt_enable = 0;
  opt.zupt_std=0.0001;
  opt.zupta_enable=0;
  opt.zupta_std=0.0001;
//  {0,-0.8 *_deg,2.1 *_deg};
  opt.gnss_std_scale = 1;
  opt.lb_gnss[0]=0.1;
  opt.lb_gnss[1]=0.3;
  NavOutput nav;
  ImuData imu;
  GnssData gnss;

  NavWriter writer{result_path, NavFileFormat::NavAscii};
  navInitialize(&opt);
  path.header.frame_id = "ins";
  path.header.stamp = ros::Time::now();
  LOG(INFO) << fmt::format("Start align,mode={},vel_threshold={}",
						   opt.align_mode,
						   opt.align_vel_threshold);//"Start align,mode = " << opt.align_mo;
  if (opt.align_mode == AlignMode::ALIGN_MOVING) {
	while (ros::ok()) {
	  if (!imu_queue.empty()) {
		imu = imu_queue.front();
		imu_queue.pop_front();
		if (navAlignLevel(&imu)) {
		  LOG(INFO) << "align finished!";
		  break;
		}
	  }
	  if (!gnss_queue.empty()) {
		gnss = gnss_queue.front();
		gnss_queue.pop_front();
		LOG(INFO) << "moving align: vel= " << navAlignGnss(&gnss);
	  }
	}
  } else {
	LOG(ERROR) << "supported align mode" << (int)opt.align_mode;
	return 1;
  }
  LOG(INFO) << "Align finished";
  navInitialize(&opt);
//  PVA_pub.publish(To_fusion_path(&nav));
  while (ros::ok()) {
	if (gnss_queue.empty()) {
	  gnss = gnss_queue.front();
	  gnss_queue.pop_front();
	  LOG(INFO) << "GNSS" << gnss;
	  if (GnssCheck(gnss))
		navSetPos(&gnss.lat, gnss.height, gnss.pos_std);
	}
	if (!vel_queue.empty()) {
	  auto vel = vel_queue.front();
	  vel_queue.pop_front();
	  navSetVel(&vel);
	}
	if (!imu_queue.empty()) {
	  imu = imu_queue.front();
	  imu_queue.pop_front();
	  timeUpdate(&imu);
	  navGetResult(&nav);
	  if (save_result)
		writer.update(nav);
	  publishResult(rst_pub, nav);
	  publishDebugInfo(debug_pub, nav);
	  if (std::isnan(nav.lat)) {
		LOG(INFO) << "System does not converge";
		break;
	  }
	}
  }
  LOG(INFO) << "result was save to:" << result_path;
  ros::shutdown();
  ros::waitForShutdown();
  LOG(INFO) << "Process finished";
  s.stop();
  return 0;
}
