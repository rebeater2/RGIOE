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
#include "fmt/format.h"
#include "glog/logging.h"

#include "NavStruct.h"
#include <thread>
#include <nav_msgs/Path.h>
#include <WGS84.h>
#include "Convert.h"
#include "boost/circular_buffer.hpp"

boost::circular_buffer<ImuData> imu_queue;
boost::circular_buffer<GnssData> gnss_queue;
boost::circular_buffer<Velocity> vel_queue;


//初始发送fusion\gnss到rviz
nav_msgs::Path ros_path_;
nav_msgs::Path ros_path_gnss;

#define FUSION_ENGINE RGIOE

/*初始经度、纬度、高程*/

bool init = false;
double base_lla[3];

extern bool GnssCheck(const GnssData &gnss);

void OnImuMsgCallBack(const navplay::imu::ConstPtr &imu_msg) {
  LOG_FIRST_N(INFO, 100) << __FUNCTION__ << fmt::format("time:{}, thread_id:", imu_msg.get()->gpst)
					   << std::this_thread::get_id();
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
static nav_msgs::Path path;
void publishResult(ros::Publisher &pub,const NavOutput &nav){
  geometry_msgs::PoseStamped pose_stamped;
  if (!init) {
    base_lla[0] = nav.lat;
    base_lla[1] = nav.lon;
    base_lla[2] = nav.height;
    init = true;
  }
  auto is_int = [ ](double a){return fabs(a-int(a))<0.01;};
  if (!is_int(nav.gpst)) {
    pub.publish(path);
    return ;
  }
  auto d = WGS84::Instance().distance(nav.lat * _deg,
									  nav.lon * _deg,
									  base_lla[0] * _deg,
									  base_lla[1] * _deg,
									  nav.height,
									  base_lla[2]);
  pose_stamped.pose.position.x = d.dn;
  pose_stamped.pose.position.y = d.de;
  pose_stamped.pose.position.z = d.dd;
  LOG(INFO)<<fmt::format("Relative position:{} {} {}",d.dn,d.de,d.dd);
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

int main(int argc, char **argv) {
  google::InitGoogleLogging("log");
  google::LogToStderr();
  imu_queue.set_capacity(10);
  gnss_queue.set_capacity(1);
  vel_queue.set_capacity(10);
  ros::init(argc, argv, "rtsim_fusion_node");
  ros::NodeHandle handle;
  ros::Subscriber imu_sub = handle.subscribe("imu_data", 10, OnImuMsgCallBack);
  ros::Subscriber gnss_sub = handle.subscribe("gnss_data", 10, OnGnssMsgCallBack);
  ros::Subscriber vel_sub = handle.subscribe("vel_data", 10, OnVelMsgCallBack);
  ros::Publisher rst_pub = handle.advertise<nav_msgs::Path>("PVA", 10);
  ros::Publisher PVA_gnss = handle.advertise<nav_msgs::Path>("gnss", 100);
  /*multi thread*/
  ros::AsyncSpinner s(3);
  s.start();
  init = false;

  Option opt = default_option;
  opt.align_mode = ALIGN_MOVING;
  opt.gnss_std_scale = 0.01;
  NavOutput nav;
  ImuData imu;
  GnssData gnss;
  path.header.frame_id="ins";
  path.header.stamp = ros::Time::now();
  LOG(INFO) << "Start align,mode = " << opt.align_mode;
  if (opt.align_mode == AlignMode::ALIGN_MOVING) {
	while (ros::ok()) {
		if(!imu_queue.empty()){
		  LOG(INFO) << "IMU data get";
		  imu = imu_queue.front();
		  imu_queue.pop_front();
		  if (navAlignLevel(&imu)) {
		    LOG(INFO) << "align finished!";
		    break;
		  };
		}
	  if (!gnss_queue.empty()) {
	    LOG(INFO) << "GNSS data get";
		gnss = gnss_queue.front();
		gnss_queue.pop_front();
		LOG(INFO)<<"vel= "<<navAlignGnss(&gnss);
	  }
	  LOG_FIRST_N(INFO,10) << "in align..." << opt.align_mode;
	}
  } else {
	LOG(ERROR) << "supported align mode" << (int)opt.align_mode;
	return 1;
  }
  LOG(INFO)<<"Align finished";
  navInitialize(&opt);
//  PVA_pub.publish(To_fusion_path(&nav));
  while (ros::ok()) {
	if (gnss_queue.empty()) {
	  gnss = gnss_queue.front();
	  gnss_queue.pop_front();
	  LOG_EVERY_N(INFO,10) << fmt::format("GNSS update @ {} {} {} {} {} {} {}",
							   gnss.gpst,
							   gnss.lat,
							   gnss.lon,
							   gnss.height,
							   gnss.pos_std[0],
							   gnss.pos_std[1],
							   gnss.pos_std[2]);//{};
	  navSetPos(&gnss.lat, gnss.height, gnss.pos_std);
	}
	if (!imu_queue.empty()) {
	  imu = imu_queue.front();
	  imu_queue.pop_front();
	  timeUpdate(&imu);
	  navGetResult(&nav);
	  publishResult(rst_pub, nav);
	  LOG(INFO)<<fmt::format("rst: lat:{:.5f} {:.8f} {:.8f} {:.4f} ",nav.gpst,nav.lat,nav.lon,nav.height);
	  LOG(INFO)<<fmt::format("imu: lat:{:.5f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} ",
							imu.gpst,imu.gyro[0],imu.gyro[1],imu.gyro[2],imu.acce[0],imu.acce[1],imu.acce[2]
							 );
	}
  }
  LOG(INFO) << "Process finished";
  s.stop();
  return 0;
}
