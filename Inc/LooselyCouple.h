/**
* @file LooselyCouple.h in LooselyCouple2020_cpp
* @author rebeater
* @comment C 语言接口，
* Create on 5/9/21 1:18 PM
* @version 1.0
**/
#ifndef LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
#define LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
#include "NavStruct.h"
#ifdef __cplusplus
extern "C" {
#endif

int kalmanInitialize();
int kalmanOutput(NavOutput *nav_output);
double kalmanAlignGnss(const GnssData *gnss);
void getXd(double *xds);/*for debug*/
/*接收前右下、非增量形式的惯导数据*/
int kalmanAlignLevel(const ImuData *imu);
void kalmanSetGNSS(const GnssData *gnss);
void kalmanSetVel(const Velocity *vel);/*里程计速度更新*/
void kalmanUpdate(const ImuData *imu);
#ifdef __cplusplus
};
#endif

#endif //LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
