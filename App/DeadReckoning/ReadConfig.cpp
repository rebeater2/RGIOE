/**
* @file ReadConfig.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 8/19/21 11:36 AM
* @version 1.0
**/


#include "ReadConfig.h"
#include "Config.h"
#include <string.h>
#include <string>
void loadYamlConfig(char *yaml_path, char *imu_path, char *gnss_path, char *out_path, RgioeOption *opt, NavOutput *nav) {
  Config cfg;
  cfg.LoadFrom(yaml_path);
  strcpy(imu_path, cfg.imu_config.file_path.c_str());
  strcpy(gnss_path, cfg.gnss_config.file_path.c_str());
  strcpy(out_path, cfg.output_path.c_str());
  std::string s;
  bool ok;
  *opt = cfg.GetOption();
};