/**
* @file navplay.cpp in Project
* @author rebeater
* @comment 数据回放，读取IMU数据并且回放成仿真
* Create on 1/9/22 1:12 PM
* @version 1.0
**/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_broadcaster.h"

#include "navplay/imu.h"
#include "navplay/gnss.h"
#include "navplay/vel.h"

#include "Earth.h"
#include "Convert.h"
#include "NavRawReader.h"
#include "glog/logging.h"
#include "fmt/format.h"

#include "AdiHandle.h"

NavRawReader *preader;
ros::Timer timer{};
double base_lla[3]{0, 0, 0};
int ConvertVelRawToFloat(const InsCube::VelocityRaw *raw, Velocity *vel) {
  vel->forward = ((float)((int16_t)(raw->vh_ << 8u) | raw->vl_)) / 1000.0f;
  vel->angular = ((float)((int16_t)(raw->ah_ << 8u) | raw->al_)) / 1000.0f;
  return 0;
}
/** @brief
 * 根据时间戳回放数据
 */
class Beats {
 private:
  nav_msgs::Path path;
  double time_elapse = 0;
  double gpst_start = 0;
  bool real_time = true;
  int speed_up = 1;/*should be 1～5*/
  ros::Time start = ros::Time::now();
  ros::NodeHandle handle;
  ros::Publisher imu_publisher = handle.advertise<navplay::imu>("imu_data", 100);
  ros::Publisher gnss_publisher = handle.advertise<navplay::gnss>("gnss_data", 1);
  ros::Publisher vel_publisher = handle.advertise<navplay::vel>("vel_data", 10);
  ros::Publisher rst_publisher = handle.advertise<nav_msgs::Path>("rst_data", 1, true);
  InsCube::RawDataDef raw{};
  bool base_set = false;
 public:
  Beats() {
	handle.param("real_time", real_time, true);
	handle.param("speed_up", speed_up, 1);
	handle.param("base_lat", base_lla[0], 1.);
	handle.param("base_lon", base_lla[1], 1.);
	handle.param("base_height", base_lla[2], 1.);
	path.header.frame_id = "ins";
	path.header.stamp = ros::Time::now();
	path.poses.reserve(1024 * 1024 * 10);
	LOG(INFO) << "real time mode:" << real_time;
	LOG(INFO) << "speed up: X" << speed_up;
	LOG(INFO) << fmt::format("base station: {} {} {}", base_lla[0], base_lla[1], base_lla[2]);
  }
 public:
  void close() {
	LOG(INFO) << "all data play finished,timer closed";
	navplay::imu imu_msg;
	imu_msg.gpst = -1;
	imu_publisher.publish(imu_msg);
	timer.stop();
  }
  navplay::gnss toGnssMsg() const {
	navplay::gnss gnss_msg;
	gnss_msg.gpst = raw.gpst;
	gnss_msg.lat = raw.gnss_.lat;
	gnss_msg.lon = raw.gnss_.lon;
	gnss_msg.h = raw.gnss_.height;
	gnss_msg.pos_std[0] = raw.gnss_.pos_std[0];
	gnss_msg.pos_std[1] = raw.gnss_.pos_std[1];
	gnss_msg.pos_std[2] = raw.gnss_.pos_std[2];
	gnss_msg.mode = raw.gnss_.mode;
	return gnss_msg;
  }
  navplay::imu toImuMsg() const {
	navplay::imu imu_msg;
	ImuData imu;
	imu_msg.gpst = raw.gpst;
	static InsCube::Adis16465Handle adis16465Handle;
	adis16465Handle.ConvertRaw2D(raw.imu_.raw_, imu);
	imu_msg.acce[1] = imu.acce[0];
	imu_msg.acce[0] = imu.acce[1];
	imu_msg.acce[2] = -imu.acce[2];
	imu_msg.gyro[1] = imu.gyro[0];
	imu_msg.gyro[0] = imu.gyro[1];
	imu_msg.gyro[2] = -imu.gyro[2];
	return imu_msg;
  }
  navplay::vel toVelMsg() const {
	navplay::vel vel_msg;
	Velocity vel;
	ConvertVelRawToFloat(&raw.vel_, &vel);
	vel_msg.gpst = raw.gpst;
	vel_msg.forward = vel.forward;
	vel_msg.angular = vel.angular;
	return vel_msg;
  }
  nav_msgs::Path toPoseMsg() {
	geometry_msgs::PoseStamped pose_stamped;
	if (!base_set) {
	  base_lla[0] = raw.rst_.lat;
	  base_lla[1] = raw.rst_.lon;
	  base_lla[2] = raw.rst_.height;
	  base_set = true;
	}
	auto is_int = [](double a) { return fabs(a - int(a)) < 0.01; };
	if (!is_int(raw.gpst)) {
	  return path;
	}
	auto d = WGS84::Instance().distance(raw.rst_.lat * _deg,
										raw.rst_.lon * _deg,
										base_lla[0] * _deg,
										base_lla[1] * _deg,
										raw.rst_.height,
										base_lla[2]);
	pose_stamped.pose.position.x = d.de;
	pose_stamped.pose.position.y = d.dn;
	pose_stamped.pose.position.z = -d.dd;
	Quad quad =
		Convert::euler_to_quaternion({raw.rst_.atti[0] * _deg, raw.rst_.atti[1] * _deg, raw.rst_.atti[2] * _deg});
//	auto quad = tf::createQuaternionFromRPY(raw.rst_.atti[0] * _deg, raw.rst_.atti[1] * _deg, raw.rst_.atti[2] * _deg);
	pose_stamped.pose.orientation.x = quad.x();
	pose_stamped.pose.orientation.y = quad.y();
	pose_stamped.pose.orientation.z = quad.z();
	pose_stamped.pose.orientation.w = quad.w();
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.header.frame_id = "ins";
	path.poses.push_back(pose_stamped);
	path.poses.size() > 10000 ? path.poses.clear() : void(0);
	return path;
  }
  /**
   * @brief 每1ms进入一次
   * @param event
   */
  void operator()(const ros::TimerEvent &event) {
	if (!preader) {
	  close();
	  return;
	}
	if (gpst_start <= 0) {
	  if (!preader->ReadNext(raw)) {
		close();
	  }
	  gpst_start = preader->GetTime();
	  start = ros::Time::now();
	  return;
	}
	/*real-time*/
	if (real_time) {
	  time_elapse = (event.current_real - start).toSec();
	} else {
	  /*加速逻辑*/
	  time_elapse += 0.001 * speed_up;
	}
	LOG_EVERY_N(INFO, 5000) << "current elapse time:" << time_elapse << " " << (int)raw.gpst;
//	if (time_elapse > raw.gpst - gpst_start) {
	switch (raw.type_) {
	  case DATA_TYPE_GNSS: gnss_publisher.publish(toGnssMsg());
		break;
	  case DATA_TYPE_IMU: imu_publisher.publish(toImuMsg());
		break;
	  case DATA_TYPE_VEL: vel_publisher.publish(toVelMsg());
		break;
	  case DATA_TYPE_RST: rst_publisher.publish(toPoseMsg());
		break;
	  case DATA_TYPE_BMP: break;/*TODO 气压计懒得写了*/
	  default: LOG(INFO) << "Error, this format is NOT defined!" << raw.type_;
		break;
	}
	if (!preader->ReadNext(raw)) {
	  close();
	}
//	}
	ros::spinOnce();
  }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging("log");
  google::LogToStderr();
  ros::init(argc, argv, "navplay_node");
  ros::NodeHandle handle;
  std::string raw_filename;
  handle.param<std::string>("raw_filename", raw_filename, "");
  LOG(INFO) << "raw file name" << raw_filename;
  handle.param<double>("base_lat", base_lla[0], 0);
  handle.param<double>("base_lon", base_lla[1], 0);
  handle.param<double>("base_height", base_lla[2], 0);
  LOG(INFO) << fmt::format("Base station:", base_lla[0], base_lla[1], base_lla[2]);
  /*打开文件准备开始读*/
  preader = new NavRawReader(raw_filename);
  if (!preader->IsOk()) {
	LOG(ERROR) << raw_filename << " is not found";
	return 1;
  }
  /*开启计时器*/
  static ros::Timer temp = handle.createTimer(ros::Duration(1.0 / 1000), Beats());
//  timer = &temp;
  timer = std::move(temp);
  geometry_msgs::Pose pos;
  ros::spin();
}

