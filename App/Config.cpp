//
// Created by rebeater on 2021/1/16.
//

#include <cstring>
#include "Config.h"
/*ImuPara default_imupara{0.0112 * _deg / _sqrt_h, 0.0025 / _sqrt_h,
						-1500 * _mGal, 1000 * _mGal, -3000 * _mGal,
						1000 * _deg / _hour, -200 * _deg / _hour, 283 * _deg / _hour,
						0, 0, 0,
						0, 0, 0,
						400 * _mGal, 400 * _mGal, 400 * _mGal,
						4.83 * _deg / _hour, 8 * _deg / _hour, 8 * _deg / _hour,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1 * _hour, 1 * _hour
};
NavPva default_pva{0, 0, 0, 0};
Option default_option{
	default_imupara, default_pva,
	128,
	AlignMode::ALIGN_USE_GIVEN,
	0, 0, 0, 0,
	1, 0.3, 0.01, 0,
	0.1, 0.3, -0.24,
	0.2, 0.35,
	0, 0, 0,
	0, 0, 0,
	0.5, 0.5, 0.9,
	0.2, 0.2, 0.2,
	0.3, 0.3, 0.3,
	0.3, 0.2,
#if KD_IN_KALMAN_FILTER == 1
	1.29, 0.3,
#endif
};*/

Config::Config(const string &yml_path) {
  root_node = YAML::LoadFile(yml_path);
  imu_filepath = root_node["imu-path"].as<string>();
  imu_para_filepath = root_node["imu-parameter-cfg"].as<string>();
  gnss_filepath = root_node["gnss-path"].as<string>();
  output_filepath = root_node["output-path"].as<string>();
  odo_filepath = root_node["odo-path"].as<string>();
  start_time = root_node["start-time"].as<double>();
  end_time = root_node["end-time"].as<double>();

  imu_format = (ImuFileFormat)root_node["imu-format"].as<int>();
  gnss_format = (GnssFileFormat)root_node["gnss-format"].as<int>();

}

Option Config::getOption() {
  Option opt{};
//    opt.gnss_format = root_node["gnss-format"].as<int>();
  opt.imuPara = getImuPara();
  opt.d_rate = root_node["imu-data-rate"].as<int>();
  for (int i = 0; i < 3; i++) {
	opt.pos_std[i] = root_node["init-pos-std"][i].as<double>();
	opt.vel_std[i] = root_node["init-vel-std"][i].as<double>();
	opt.atti_std[i] = root_node["init-atti-std"][i].as<double>() * _deg;
	opt.angle_bv[i] = root_node["install-angle"][i].as<double>() * _deg;
	opt.lb_wheel[i] = root_node["odo-level-arm"][i].as<double>();
	opt.lb_gnss[i] = root_node["antenna-level-arm"][i].as<double>();
  }
  opt.align_mode = (AlignMode)root_node["alignment-mode"].as<int>();
  opt.nhc_enable = root_node["nhc-enable"].as<bool>();
  opt.nhc_std[0] = root_node["nhc-std"][0].as<double>();
  opt.nhc_std[1] = root_node["nhc-std"][1].as<double>();

  opt.zupt_enable = root_node["zupt-enable"].as<bool>();
  opt.zupta_enable = root_node["zupta-enable"].as<bool>();

  opt.outage_enable = root_node["outage-enable"].as<bool>();
#if USE_OUTAGE == 1
  opt.outage_start = root_node["outage-start"].as<int>();
  opt.outage_stop = root_node["outage-stop"].as<int>();
  opt.outage_time = root_node["outage-time"].as<int>();
  opt.outage_step = root_node["outage-step"].as<int>();
#endif
  opt.d_rate = root_node["imu-data-rate"].as<int>();
#if KD_IN_KALMAN_FILTER == 1
  opt.kd_std = root_node["odo-kd-std"].as<float>();
  opt.kd_init = root_node["odo-kd-init"].as<float>();
#endif
  return opt;
}

ImuPara Config::getImuPara() const {
  ImuPara imuPara{};
  YAML::Node imu_para_node = YAML::LoadFile(imu_para_filepath);
//    imu_para_node.IsDefined()
  imuPara.arw = imu_para_node["arw"].as<double>() * _deg / _sqrt_h;
  imuPara.vrw = imu_para_node["vrw"].as<double>() / _sqrt_h;
  for (int i = 0; i < 3; i++) {
    imuPara.gb_std[i] = 0.1 *imu_para_node["gb-std"][i].as<double>() * _deg / _hour;
	imuPara.gs_std[i] = imu_para_node["gs-std"][i].as<double>() * _ppm;
	imuPara.as_std[i] = imu_para_node["as-std"][i].as<double>() * _ppm;
	imuPara.ab_std[i] = 0.002 * imu_para_node["ab-std"][i].as<double>() * _mGal;

	imuPara.gb_ini[i] = imu_para_node["gb-ini"][i].as<double>() * _deg / _hour;
	imuPara.gs_ini[i] = imu_para_node["gs-ini"][i].as<double>() * _ppm;
	imuPara.ab_ini[i] = imu_para_node["ab-ini"][i].as<double>() * _mGal;
	imuPara.as_ini[i] = imu_para_node["as-ini"][i].as<double>() * _ppm;
  }
  imuPara.gt_corr = imu_para_node["gt-corr"].as<double>() * _hour;
  imuPara.at_corr = imu_para_node["at-corr"].as<double>() * _hour;

  return imuPara;
}

NavOutput Config::getInitNav() {
  static NavOutput nav;
  nav.week = root_node["alignment-epoch"][0].as<int>();
  nav.gpst = root_node["alignment-epoch"][1].as<double>();
  nav.lat = root_node["alignment-epoch"][2].as<double>();
  nav.lon = root_node["alignment-epoch"][3].as<double>();
  nav.height = root_node["alignment-epoch"][4].as<double>();

  nav.vn[0] = root_node["alignment-epoch"][5].as<double>();
  nav.vn[1] = root_node["alignment-epoch"][6].as<double>();
  nav.vn[2] = root_node["alignment-epoch"][7].as<double>();

  nav.atti[0] = root_node["alignment-epoch"][8].as<double>();
  nav.atti[1] = root_node["alignment-epoch"][9].as<double>();
  nav.atti[2] = root_node["alignment-epoch"][10].as<double>();

  nav.info = {0x01, 0};
  return nav;
}

