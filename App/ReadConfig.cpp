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
void loadYamlConfig(char *yaml_path, char *imu_path, char *gnss_path, char *out_path, Option *opt, NavOutput *nav) {
  Config cfg = Config(yaml_path);
  strcpy(imu_path, cfg.imu_filepath.c_str());
  strcpy(gnss_path, cfg.gnss_filepath.c_str());
  strcpy(out_path, cfg.output_filepath.c_str());
  *opt = cfg.getOption();
  if (nav != NULL) {
    *nav = cfg.getInitNav();
  }
};