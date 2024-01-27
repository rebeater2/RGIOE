/**
* @file ReadConfig.h in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 8/19/21 11:36 AM
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_APP_READCONFIG_H_
#define LOOSELYCOUPLE2020_CPP_APP_READCONFIG_H_
#ifdef __cplusplus
extern "C"{
#endif
#include "RgioeDataType.h"
  void loadYamlConfig(char *yaml_path, char *imu_path, char *gnss_path, char *out_path, RgioeOption *opt, NavOutput *nav) ;
#ifdef __cplusplus
}
#endif
#endif //LOOSELYCOUPLE2020_CPP_APP_READCONFIG_H_
