/**
* @file LooselyCouple.h in LooselyCouple2020_cpp
* @author rebeater
* @comment C 语言接口，
* Create on 5/9/21 1:18 PM
* @version 1.0
**/
#ifndef LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
#define LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
#include "Define.h"
#include "NavStruct.h"
#ifdef __cplusplus
extern "C" {
#endif
extern Option  default_option;
int navInitialize(const Option *opt);
  int navGetResult(NavOutput *pva) ;
double navAlignGnss(const GnssData *gnss);
int navAlignUseGiven(NavOutput *nav,Option *opt);
void getXd(double *xds);/*for debug*/
/*接收前右下、非增量形式的惯导数据*/
int navAlignLevel(const ImuData *imu);
void navSetPos(const double latLon[2], float h, const float std[3]);
void navSetVel(const Velocity *vel);/*里程计速度更新*/
void timeUpdate(const ImuData *imu);
#ifdef __cplusplus
};
#endif

#endif //LOOSELYCOUPLE2020_CPP_LOOSELYCOUPLE_H
