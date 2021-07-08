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
double kalmanAlignPos(const GnssData *gnss,const  ImuData *imu);
void kalmanSetGNSS(const GnssData *gnss);
void kalmanUpdate(const ImuData *imu);
void kalmanInitialize(const NavOutput *nav,const  Option *opt);
#ifdef __cplusplus
};
#endif


#endif //LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
