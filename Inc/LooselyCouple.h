/**
* @file LooselyCouple.h in LooselyCouple2020_cpp
* @author rebeater
* @comment C 语言接口，
* Create on 5/9/21 1:18 PM
* @version 1.0
**/

//
// Created by rebeater on 5/9/21.
//

#ifndef LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
#define LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H


#include "nav_struct.h"

#ifdef __cplusplus
extern "C" {
#endif


#if USE_YAML == 1
void loadYamlConfig(char *yaml_path,char *imu_path,char *gnss_path,char *out_path,Option *opt,NavOutput *nav);
#endif
int kalmanOutput(NavOutput *nav_output);
double kalmanAlignPos(GnssData *gnss, ImuData *imu);
void kalmanSetGNSS(GnssData *gnss);
void kalmanUpdate(ImuData *imu);
void kalmanInitialize(NavOutput *nav, Option *opt);
#ifdef __cplusplus
};
#endif


#endif //LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
