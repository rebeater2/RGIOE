/******************************************************************************
 * RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based
 * on Extend Kalman Filter
 * Copyright (C) 2024                                                         *
 * Author : rebeater                                                          *
 * Contact : rebeater@qq.com                                                  *
 ******************************************************************************/

#include "rgioe.h"              /*! Main fusion algorithm */
#include "FileIO.h"             /*! File read and write */
#include "Config.h"             /*! Configure file read and write */
#include "DataManager.h"        /*! Sort and manage data pointer */
#include "Outage.h"             /*! GNSS outage test */
#include "Timer.h"              /*! Run time elapse calc */

#if ENABLE_FUSION_RECORDER
#include "Recorder.h"  /*! Recorder debug infomation */
#endif

#include <glog/logging.h>       /*! Google glog for log output */

char rgioe_help_info[] = "usage: PostDataFusion config.yml";

void ShowFusionConfig(const char *argv0);

void InitialLog(const char *argv0);

/* 模块日志打印函数 */
int rgioe_log_impl(const char *fun,int line, const char *format, ...);

int main(int argc, char **argv) {
    InitialLog(argv[0]);
    ShowFusionConfig(argv[0]);
    if (argc < 2) {
        LOG(INFO) << rgioe_help_info << endl;
        return 1;
    }
#if ENABLE_FUSION_RECORDER
    Recorder::GetInstance().Initialize(argv[0]);
#endif
    /*! main action */
    Config config;
    config.LoadFrom(argv[1]);
    LOG(INFO) << config.ToStdString();
    LOG(INFO) << config.GetOption().imuPara;

    std::shared_ptr<BaseData_t> data = nullptr;
    /* buffer for rgioe */
    auto *rgioe_dev = new uint8_t[rgioe_buffer_size];
    rgioe_nav_pva_t result;
    PvaWriter writer{config.output_config.file_path, config.output_config.format};
    RgioeOption opt = config.GetOption();
    Outage outage = Outage(config.outage_config.start, config.outage_config.stop, config.outage_config.outage,
                           config.outage_config.step);

    DataManager manager;
    manager.AddFile(config.gnss_config)
            .AddFile(config.imu_config);
    manager.MoveToTime(config.start_time);

    rgioe_init(rgioe_dev, &opt, &config.align_config.init_pva);
    Timer timer;
    timer.reset();
    RgioeImuData rgioe_imu = {};
    do {
        /*! step1: read raw data*/
        data = manager.GetNextData();
        if (!data) break;
//        LOG_IF(INFO, data->type == 2) << fmt::format("type {}:{:6f}", data->type, data->time);
        /*! step2: predict or measure update */
        switch (data->type) {
            case DATA_TYPE_IMU: {
                auto imu = std::dynamic_pointer_cast<ImuData_t>(data);
                rgioe_imu = imu->toRgioeData();
                rgioe_timeupdate(rgioe_dev, data->time, &rgioe_imu);
            }
                break;
            case DATA_TYPE_GNSS: {
                auto gnss = std::dynamic_pointer_cast<GnssData_t>(data);
                auto rgioe_gnss = gnss->toRgioeData();
                if (config.outage_config.enable and outage.IsOutage(gnss->time))
                    break;
                rgioe_gnssupdate(rgioe_dev, rgioe_imu.gpst, &rgioe_gnss);
            }
                break;
            default:
                break;
        }
        /*! step3: output the result (TODO configure output rate)*/
        if (rgioe_get_status(rgioe_dev) == RGIOE_STATUS_NAVIGATION || true) {
            rgioe_get_result(rgioe_dev, &result);
            writer.Write(result);
            LOG_EVERY_N(INFO,opt.d_rate * 1000) << "forward progress:" <<manager.GetProgress() << "%";
        }
    } while (true);
    LOG(INFO) << "Time usage:" << timer.elapsed() << "ms";
    LOG(INFO) << "Heap size: " << rgioe_buffer_size;
    LOG(INFO) << "Final nav:" << result;
    LOG(INFO) << "Result was saved to:" << config.output_config.file_path;
#if ENABLE_FUSION_RECORDER
    LOG(INFO) << "Recorder was saved to:" << Recorder::GetInstance().GetRcdFilename();
#endif
    rgioe_deinit(rgioe_dev);
    delete[]rgioe_dev;
}

void InitialLog(const char *argv0) {
    google::InitGoogleLogging(argv0);
    FLAGS_alsologtostderr = true;
}


void ShowFusionConfig(const char *argv0) {
    LOG(INFO) << "---------------------start of config---------------------------";
    (void *) &argv0;
    LOG(INFO) << CopyRight;
    LOG(INFO) << "Build information:" << rgioe_build_info;
#define SHOW_MACRO(macro)   LOG(INFO) << #macro << " = " << (macro?"ON":"OFF")
    SHOW_MACRO(RGIOE_ESTIMATE_ACCE_SCALE_FACTOR);
    SHOW_MACRO(RGIOE_ESTIMATE_GYRO_SCALE_FACTOR);
    SHOW_MACRO(RGIOE_ESTIMATE_GNSS_LEVEL_ARM);
    SHOW_MACRO(RGIOE_ESTIMATE_ODOMETER_SCALE_FACTOR);
    SHOW_MACRO(ENABLE_FUSION_RECORDER);
    LOG(INFO) << "State vector size:" << STATE_CNT;
#undef SHOW_MACRO
    LOG(INFO) << "-----------------------end of config-----------------------";
}

#include <cstdarg>
extern int rgioe_log_impl(const char *fun,int line, const char *format, ...) {
    char buffer[1024];
    int retval = sprintf(buffer,"=>[%s:%d] ",fun,line);
    std::va_list ap;
    va_start(ap,format);
    retval = vsprintf((char *) buffer+ retval, format, ap);
    va_end(ap);
    LOG(INFO) << buffer;
    return retval;
}