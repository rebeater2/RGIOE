/**
* @file config.cpp in UiDataFusion
* @author rebeater
* @comment
* Create on 11/24/21 10:25 AM
* @version 1.0
**/


#include "Config.h"
#include "yaml-cpp/yaml.h"
#include "fmt/format.h"
#include <fstream>
#include <string>
using namespace std;
void SaveConfig(const Config &cfg, const string &path) {
  YAML::Node node;
  node["IMU-Config"]["enable"] = cfg.imu_config.enable;
  node["IMU-Config"]["file-path"] = cfg.imu_config.file_path;
  node["IMU-Config"]["d-rate"] = cfg.imu_config.d_rate;
  node["IMU-Config"]["format"] = (int)cfg.imu_config.format;
  node["IMU-Config"]["parameter-path"] = cfg.imu_config.parameter_path;
  node["IMU-Config"]["frame"] = (int)cfg.imu_config.frame;

  node["GNSS-Config"]["enable"] = cfg.gnss_config.enable;
  node["GNSS-Config"]["file-path"] = cfg.gnss_config.file_path;
  node["GNSS-Config"]["format"] = (int)cfg.gnss_config.format;
  for (float i:cfg.gnss_config.level_arm)
	node["GNSS-Config"]["level-arm"].push_back(i);
  node["GNSS-Config"]["double-antenna-enable"] = cfg.gnss_config.double_antenna_enable;
  node["GNSS-Config"]["antenna-pitch"] = cfg.gnss_config.pitch_of_antenna;
  node["GNSS-Config"]["antenna-yaw"] = cfg.gnss_config.yaw_of_antenna;

  node["Odometer"]["enable"] = cfg.odometer_config.enable;
  node["Odometer"]["odometer-std"] = cfg.odometer_config.odometer_std;
  node["Odometer"]["file-path"] = cfg.odometer_config.file_path;
  node["Odometer"]["NHC-enable"] = cfg.odometer_config.nhc_enable;
  node["Odometer"]["NHC-std"][0] = cfg.odometer_config.nhc_std[0];
  node["Odometer"]["NHC-std"][1] = cfg.odometer_config.nhc_std[1];
  for (float i:cfg.odometer_config.angle_bv) node["Odometer"]["angle-bv"].push_back(i);
  for (float i:cfg.odometer_config.wheel_level_arm) node["Odometer"]["wheel-level-arm"].push_back(i);

  for (int i : cfg.gnss_config.index)
	node["GNSS-Config"]["columns"].push_back(i);

  node["ZUPT-Config"]["ZUPT-enable"] = cfg.zupt_config.zupt_enable;
  node["ZUPT-Config"]["ZUPTA-enable"] = cfg.zupt_config.zupt_enable;
  node["ZUPT-Config"]["zupt-std"] = cfg.zupt_config.zupt_std;
  node["ZUPT-Config"]["zupta-std"] = cfg.zupt_config.zupta_std;
  node["ZUPT-Config"]["zupt-window"] = cfg.zupt_config.zupt_window;
  node["ZUPT-Config"]["static-wide"] = cfg.zupt_config.static_wide;
  node["ZUPT-Config"]["threshold"] = cfg.zupt_config.threshold;

  node["Align-Config"]["mode"] = (int)cfg.align_config.mode;
  node["Align-Config"]["velocity-threshold"] = (int)cfg.align_config.vel_threshold_for_moving;
  node["Align-Config"]["init-PVA"]["week"] = (int)cfg.align_config.init_pva.week;
  node["Align-Config"]["init-PVA"]["gpst"] = cfg.align_config.init_pva.gpst;
  node["Align-Config"]["init-PVA"]["lat"] = cfg.align_config.init_pva.lat;
  node["Align-Config"]["init-PVA"]["lon"] = cfg.align_config.init_pva.lon;
  node["Align-Config"]["init-PVA"]["height"] = cfg.align_config.init_pva.height;
  node["Align-Config"]["init-PVA"]["vn"][0] = cfg.align_config.init_pva.vn[0];
  node["Align-Config"]["init-PVA"]["vn"][1] = cfg.align_config.init_pva.vn[1];
  node["Align-Config"]["init-PVA"]["vn"][2] = cfg.align_config.init_pva.vn[2];
  node["Align-Config"]["init-PVA"]["atti"][0] = cfg.align_config.init_pva.atti[0];
  node["Align-Config"]["init-PVA"]["atti"][1] = cfg.align_config.init_pva.atti[1];
  node["Align-Config"]["init-PVA"]["atti"][2] = cfg.align_config.init_pva.atti[2];

  node["Outage-Config"]["enable"] = cfg.outage_config.enable;
  node["Outage-Config"]["start"] = cfg.outage_config.start;
  node["Outage-Config"]["stop"] = cfg.outage_config.stop;
  node["Outage-Config"]["step"] = cfg.outage_config.step;
  node["Outage-Config"]["outage"] = cfg.outage_config.outage;

  node.SetStyle(YAML::EmitterStyle::Block);
  node.SetTag(path);
  ofstream out(path);
  out << node;
  out.close();
}

/*int main() {
  Config config;
  config.odometer_config.file_path = "hello odometer.txt";
  string path = "../config.yml";
  SaveConfig(config, path);

}*/
void Config::SaveTo(const string &path) const {
  YAML::Node node;

  node["IMU-Config"]["enable"] = imu_config.enable;
  node["IMU-Config"]["file-path"] = imu_config.file_path;
  node["IMU-Config"]["d-rate"] = imu_config.d_rate;
  node["IMU-Config"]["format"] = (int)imu_config.format;
  node["IMU-Config"]["parameter-path"] = imu_config.parameter_path;
  node["IMU-Config"]["frame"] = (int)imu_config.frame;

  node["GNSS-Config"]["enable"] = gnss_config.enable;
  node["GNSS-Config"]["file-path"] = gnss_config.file_path;
  node["GNSS-Config"]["format"] = (int)gnss_config.format;
  for (float i:gnss_config.level_arm)
	node["GNSS-Config"]["level-arm"].push_back(i);
  node["GNSS-Config"]["double-antenna-enable"] = gnss_config.double_antenna_enable;
  node["GNSS-Config"]["antenna-pitch"] = gnss_config.pitch_of_antenna;
  node["GNSS-Config"]["antenna-yaw"] = gnss_config.yaw_of_antenna;

  node["Odometer"]["enable"] = odometer_config.enable;
  node["Odometer"]["odometer-std"] = odometer_config.odometer_std;
  node["Odometer"]["file-path"] = odometer_config.file_path;
  node["Odometer"]["NHC-enable"] = odometer_config.nhc_enable;
  node["Odometer"]["NHC-std"][0] = odometer_config.nhc_std[0];
  node["Odometer"]["NHC-std"][1] = odometer_config.nhc_std[1];
  for (float i:odometer_config.angle_bv) node["Odometer"]["angle-bv"].push_back(i);
  for (float i:odometer_config.wheel_level_arm) node["Odometer"]["wheel-level-arm"].push_back(i);


  node["Odometer"]["scale-factor"]= odometer_config.scale_factor;
  node["Odometer"]["scale-factor-std"]= odometer_config.scale_factor_std;
  for (int i : gnss_config.index)
	node["GNSS-Config"]["columns"].push_back(i);

  node["ZUPT-Config"]["ZUPT-enable"] = zupt_config.zupt_enable;
  node["ZUPT-Config"]["ZUPTA-enable"] = zupt_config.zupta_enable;
  node["ZUPT-Config"]["zupt-std"] = zupt_config.zupt_std;
  node["ZUPT-Config"]["zupta-std"] = zupt_config.zupta_std;
  node["ZUPT-Config"]["zupt-window"] = zupt_config.zupt_window;
  node["ZUPT-Config"]["static-wide"] = zupt_config.static_wide;
  node["ZUPT-Config"]["threshold"] = zupt_config.threshold;

  node["Align-Config"]["mode"] = (int)align_config.mode;
  node["Align-Config"]["velocity-threshold"] = (int)align_config.vel_threshold_for_moving;
  node["Align-Config"]["init-PVA"]["week"] = (int)align_config.init_pva.week;
  node["Align-Config"]["init-PVA"]["gpst"] = align_config.init_pva.gpst;
  node["Align-Config"]["init-PVA"]["lat"] = align_config.init_pva.lat;
  node["Align-Config"]["init-PVA"]["lon"] = align_config.init_pva.lon;
  node["Align-Config"]["init-PVA"]["height"] = align_config.init_pva.height;
  node["Align-Config"]["init-PVA"]["vn"][0] = align_config.init_pva.vn[0];
  node["Align-Config"]["init-PVA"]["vn"][1] = align_config.init_pva.vn[1];
  node["Align-Config"]["init-PVA"]["vn"][2] = align_config.init_pva.vn[2];
  node["Align-Config"]["init-PVA"]["atti"][0] = align_config.init_pva.atti[0];
  node["Align-Config"]["init-PVA"]["atti"][1] = align_config.init_pva.atti[1];
  node["Align-Config"]["init-PVA"]["atti"][2] = align_config.init_pva.atti[2];

  node["Outage-Config"]["enable"] = outage_config.enable;
  node["Outage-Config"]["start"] = outage_config.start;
  node["Outage-Config"]["stop"] = outage_config.stop;
  node["Outage-Config"]["step"] = outage_config.step;
  node["Outage-Config"]["outage"] = outage_config.outage;

  node["start-time"] = start_time;
  node["stop-time"] = stop_time;
  node["output-path"] = output_path;

  node.SetStyle(YAML::EmitterStyle::Block);
  node.SetTag(path);
  ofstream out(path);
  out << node;
  out.close();
}
void Config::LoadFrom(const string &path) {
  YAML::Node node = YAML::LoadFile(path);
  start_time = node["start-time"].as<float>();
  stop_time = node["stop-time"].as<float>();
  output_path = node["output-path"].as<string>();
  enable_rts = node["enable-rts"].as<bool>();
  imu_config.enable = node["IMU-Config"]["enable"].as<bool>();
  imu_config.file_path = node["IMU-Config"]["file-path"].as<std::string>();
  imu_config.d_rate = node["IMU-Config"]["d-rate"].as<int>();
  imu_config.format = (IMUFileFormat)node["IMU-Config"]["format"].as<int>();
  imu_config.parameter_path = node["IMU-Config"]["parameter-path"].as<std::string>();
  imu_config.frame = (IMUFrame)node["IMU-Config"]["frame"].as<int>();

  gnss_config.enable = node["GNSS-Config"]["enable"].as<bool>();
  gnss_config.file_path = node["GNSS-Config"]["file-path"].as<std::string>();
  gnss_config.format = (GnssFileFormat)node["GNSS-Config"]["format"].as<int>();
  gnss_config.scale_of_std = node["GNSS-Config"]["std-scale"].as<float>();
  for (int i = 0; i < 3; i++)
	gnss_config.level_arm[i] = node["GNSS-Config"]["level-arm"][i].as<float>();

  gnss_config.double_antenna_enable = node["GNSS-Config"]["double-antenna-enable"].as<bool>();
  gnss_config.pitch_of_antenna = node["GNSS-Config"]["antenna-pitch"].as<float>();
  gnss_config.yaw_of_antenna = node["GNSS-Config"]["antenna-yaw"].as<float>();

  odometer_config.enable = node["Odometer"]["enable"].as<bool>();
  odometer_config.odometer_std = node["Odometer"]["odometer-std"].as<float>();
  odometer_config.file_path = node["Odometer"]["file-path"].as<std::string>();
  odometer_config.nhc_enable = node["Odometer"]["NHC-enable"].as<bool>();
  odometer_config.nhc_std[0] = node["Odometer"]["NHC-std"][0].as<float>();
  odometer_config.nhc_std[1] = node["Odometer"]["NHC-std"][1].as<float>();
  for (int i = 0; i < 3; i++) {
	odometer_config.angle_bv[i] = node["Odometer"]["angle-bv"][i].as<float>();
  }
  for (int i = 0; i < 3; i++) {
	odometer_config.wheel_level_arm[i] = node["Odometer"]["wheel-level-arm"][i].as<float>();
  }
  odometer_config.scale_factor = node["Odometer"]["scale-factor"].as<float>();
  odometer_config.scale_factor_std = node["Odometer"]["scale-factor-std"].as<float>();
  for (int i = 0; i < 3; i++) {
	gnss_config.index[i] = node["GNSS-Config"]["columns"][i].as<int>();
  }
  zupt_config.zupt_enable = node["ZUPT-Config"]["ZUPT-enable"].as<bool>();
  zupt_config.zupta_enable = node["ZUPT-Config"]["ZUPTA-enable"].as<bool>();
  zupt_config.zupt_std = node["ZUPT-Config"]["zupt-std"].as<float>();
  zupt_config.zupta_std = node["ZUPT-Config"]["zupta-std"].as<float>();
  zupt_config.zupt_window = node["ZUPT-Config"]["zupt-window"].as<int>();
  zupt_config.static_wide = node["ZUPT-Config"]["static-wide"].as<int>();
  zupt_config.threshold = node["ZUPT-Config"]["threshold"].as<float>();

  align_config.mode = (AlignMode)node["Align-Config"]["mode"].as<int>();
  align_config.vel_threshold_for_moving = node["Align-Config"]["velocity-threshold"].as<float>();
  align_config.init_pva.week = node["Align-Config"]["init-PVA"]["week"].as<int>();
  align_config.init_pva.gpst = node["Align-Config"]["init-PVA"]["gpst"].as<double>();
  align_config.init_pva.lat = node["Align-Config"]["init-PVA"]["lat"].as<double>();
  align_config.init_pva.lon = node["Align-Config"]["init-PVA"]["lon"].as<double>();
  align_config.init_pva.height = node["Align-Config"]["init-PVA"]["height"].as<float>();

  align_config.init_pva.vn[0] = node["Align-Config"]["init-PVA"]["vn"][0].as<float>();
  align_config.init_pva.vn[1] = node["Align-Config"]["init-PVA"]["vn"][1].as<float>();
  align_config.init_pva.vn[2] = node["Align-Config"]["init-PVA"]["vn"][2].as<float>();
  align_config.init_pva.atti[0] = node["Align-Config"]["init-PVA"]["atti"][0].as<float>();
  align_config.init_pva.atti[1] = node["Align-Config"]["init-PVA"]["atti"][1].as<float>();
  align_config.init_pva.atti[2] = node["Align-Config"]["init-PVA"]["atti"][2].as<float>();

  outage_config.enable = node["Outage-Config"]["enable"].as<bool>();
  outage_config.start = node["Outage-Config"]["start"].as<float>();
  outage_config.stop = node["Outage-Config"]["stop"].as<float>();
  outage_config.step = node["Outage-Config"]["step"].as<float>();
  outage_config.outage = node["Outage-Config"]["step"].as<float>();
}
Option Config::GetOption() const {
  Option opt{
	  .imuPara= imu_config.para,
	  .init_epoch = align_config.init_pva,
	  .d_rate = imu_config.d_rate,
	  .align_mode = align_config.mode,

	  .nhc_enable = odometer_config.nhc_enable,
	  .zupt_enable = zupt_config.zupt_enable,
	  .zupta_enable = zupt_config.zupta_enable,
	  .odo_enable=odometer_config.enable,

	  .zupt_std = zupt_config.zupt_std,
	  .zupta_std = zupt_config.zupta_std,
	  .lb_gnss = {gnss_config.level_arm[0], gnss_config.level_arm[1], gnss_config.level_arm[2]},
	  .odo_std = odometer_config.odometer_std,
	  .lb_wheel = {odometer_config.wheel_level_arm[0], odometer_config.wheel_level_arm[1],
				   odometer_config.wheel_level_arm[2]},
	  .angle_bv = {odometer_config.angle_bv[0] * (float)_deg, odometer_config.angle_bv[1] * (float)_deg,
				   odometer_config.angle_bv[2] * (float)_deg},
	  .nhc_std =  {odometer_config.nhc_std[0], odometer_config.nhc_std[1]},
	  .kd_init = odometer_config.scale_factor,
	  .kd_std = odometer_config.scale_factor_std,
	  .gnss_std_scale = gnss_config.scale_of_std,
	  .enable_rts = enable_rts,
  };
  return opt;
}
bool Config::LoadImuPara(string &error_msg) {
  try {
	ImuPara imuPara{};
	YAML::Node imu_para_node = YAML::LoadFile(imu_config.parameter_path);
	//    imu_para_node.IsDefined()
	imuPara.arw = (float)(imu_para_node["arw"].as<float>() * _deg / _sqrt_h);
	imuPara.vrw = imu_para_node["vrw"].as<float>() / _sqrt_h;
	for (int i = 0; i < 3; i++) {
	  imuPara.gb_std[i] = (float)(imu_para_node["gb-std"][i].as<float>() * _deg / _hour);
	  imuPara.gs_std[i] = (float)(imu_para_node["gs-std"][i].as<float>() * _ppm);
	  imuPara.as_std[i] = (float)(imu_para_node["as-std"][i].as<float>() * _ppm);
	  imuPara.ab_std[i] = (float)(imu_para_node["ab-std"][i].as<float>() * _mGal);

	  imuPara.gb_ini[i] = (float)(imu_para_node["gb-ini"][i].as<float>() * _deg / _hour);
	  imuPara.gs_ini[i] = (float)(imu_para_node["gs-ini"][i].as<float>() * _ppm);
	  imuPara.ab_ini[i] = (float)(imu_para_node["ab-ini"][i].as<float>() * _mGal);
	  imuPara.as_ini[i] = (float)(imu_para_node["as-ini"][i].as<float>() * _ppm);
	}
	imuPara.gt_corr = imu_para_node["gt-corr"].as<float>() * _hour;
	imuPara.at_corr = imu_para_node["at-corr"].as<float>() * _hour;
	imu_config.para = imuPara;
	error_msg = "";
	return true;
  } catch (exception &e) {
	error_msg = string{e.what()};
	return false;
  }
}
std::string Config::ToStdString() const {
  /*TODO*/
  string res;
  res.reserve(1024);
  res += fmt::format("imu file: {}\n",imu_config.file_path);
  res += fmt::format("imu rate: {}\n",imu_config.d_rate);
  res += fmt::format("imu format: {}\n",imu_config.format);
  res += fmt::format("imu frame: {}\n",imu_config.frame);
  res += fmt::format("gnss scale: {:3f}\n",gnss_config.scale_of_std);
  return res;
}

