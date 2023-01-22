//
// Created by rebeater on 2020/8/12.
//
#include "NavLog.h"

void logInit(char *argv, const char *path) {
#if GLOG_OUTPUT == 1
    google::InitGoogleLogging((const char *) argv);
    // Set whether log messages go to stderr instead of logfiles
//    DECLARE_bool(logtostderr);
    FLAGS_alsologtostderr = true;
// Set whether log messages go to stderr in addition to logfiles.
//    DECLARE_bool(alsologtostderr);
    FLAGS_colorlogtostderr = true;

// Set color messages logged to stderr (if supported by terminal).
//    DECLARE_bool(colorlogtostderr);

    google::SetLogDestination(0, path);
#else
    std::cout << "glog is disabled, log to console..." << std::endl;
#endif
}

void log(int level, const char *fmt, ...) {
    va_list st;
    va_start(st, fmt);
    char buffer[100] = {0};
    vsprintf(buffer, fmt, st);
    va_end(st);
#ifdef GLOG_OUTPUT
    switch (level) {
        case 0:
            LOG(INFO) << buffer << "\n";
            break;
        case 1:
            LOG(WARNING) << buffer << "\n";
            break;
        case 2:
            LOG(ERROR) << buffer << "\n";
            break;
        case 3:
            LOG(FATAL) << buffer << "\n";
            break;
        default:
            LOG(INFO) << buffer << "\n";
            break;
    }
#else
    const std::string  log_level[4] = {"INFO","WARNING","ERROR","FETAL"};
    now_time = time(nullptr);
    tm *t_tm = localtime(&now_time);
    sprintf(timebuff,"\n%04d/%02d/%02d %02d:%02d%02d",t_tm->tm_year+1900,t_tm->tm_mon+1,t_tm->tm_mday,t_tm->tm_hour,t_tm->tm_min,t_tm->tm_sec);
    if(level>=4){
        std::cout<<"level should be 1~4"<<std::endl;
        return;
    }
    std::cout << timebuff<<"\t" << log_level[level] << ':' << buffer;
#endif
}

time_t now_time;