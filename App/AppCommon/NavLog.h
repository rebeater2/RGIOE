#ifndef NAV_LOG_H__
#define NAV_LOG_H__

#include "Define.h"

#if GLOG_OUTPUT == 1

#include <glog/logging.h>

using namespace google;
#else

#include<iostream>
#include<ctime>
using namespace  std;
#endif

#include <cstdarg>

void logInit(char *argv, const char *path);

static char timebuff[100];

void log(int level, const char *fmt, ...);

#if GLOG_OUTPUT == 1
#define logi (LOG(INFO))
#define logw (LOG(WARNING))
#define loge (LOG(ERROR))
#define logf (LOG(FATAL))
#else
extern time_t now_time;
#define logi     {time_t now_time0 = time(nullptr);\
tm *t_tm0 = localtime(&now_time0);\
sprintf(timebuff,"\n%04d/%02d/%02d %02d:%02d:%02d",t_tm0->tm_year+1900,t_tm0->tm_mon+1,t_tm0->tm_mday,t_tm0->tm_hour,t_tm0->tm_min,t_tm0->tm_sec);\
}\
std::cout << timebuff<<"\t" << "INFO" << ":\t"\


#define logw  {time_t now_time1 = time(nullptr);\
tm *t_tm1 = localtime(&now_time1);\
sprintf(timebuff,"\n%04d/%02d/%02d %02d:%02d:%02d",t_tm1->tm_year+1900,t_tm1->tm_mon+1,t_tm1->tm_mday,t_tm1->tm_hour,t_tm1->tm_min,t_tm1->tm_sec);\
}\
std::cout << timebuff<<"\t" << "WARNING" << ":\t"

#define loge { \
time_t now_time2 = time(nullptr);\
tm *t_tm2 = localtime(&now_time2);\
sprintf(timebuff,"\n%04d/%02d/%02d %02d:%02d:%02d",t_tm2->tm_year+1900,t_tm2->tm_mon+1,t_tm2->tm_mday,t_tm2->tm_hour,t_tm2->tm_min,t_tm2->tm_sec);\
}\
std::cout << timebuff<<"\t" << "ERROR" <<  ":\t"

#define logf {\
time_t now_time3 = time(nullptr);\
tm *t_tm3 = localtime(&now_time3);\
sprintf(timebuff,"\n%04d/%02d/%02d %02d:%02d:%02d",t_tm3->tm_year+1900,t_tm3->tm_mon+1,t_tm3->tm_mday,t_tm3->tm_hour,t_tm3->tm_min,t_tm3->tm_sec);\
}\
std::cout << timebuff<<"\t" << "FATAL" <<  ":\t"
/*TODO logf本应该使得程序退出，但是这里没有实现*/
#endif
#endif