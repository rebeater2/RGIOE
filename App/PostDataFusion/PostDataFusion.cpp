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
#include "Recorder/Recorder.h"  /*! Recorder debug infomation */
#endif

#include <fmt/format.h>
#include <glog/logging.h>       /*! Google glog for log output */

char rgioe_help_info[] = "usage: PostDataFusion config.yml";

void ShowFusionConfig(const char *argv0);

void InitialLog(const char *argv0);

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
    do {
        /*! step1: read raw data*/
        data = manager.GetNextData();
        if (!data) break;
//        LOG_IF(INFO, data->type == 2) << fmt::format("type {}:{:6f}", data->type, data->time);
        /*! step2: predict or measure update */
        switch (data->type) {
            case DATA_TYPE_IMU: {
                auto imu = std::dynamic_pointer_cast<ImuData_t>(data);
                auto rgioe_imu = imu->toRgioeData();
                rgioe_timeupdate(rgioe_dev, data->time, &rgioe_imu);
            }
                break;
            case DATA_TYPE_GNSS: {
                auto gnss = std::dynamic_pointer_cast<GnssData_t>(data);
                auto rgioe_gnss = gnss->toRgioeData();
                if (config.outage_config.enable and outage.IsOutage(gnss->time))
                    break;
                rgioe_gnssupdate(rgioe_dev, gnss->time, &rgioe_gnss);
            }
                break;
            default:
                break;
        }
        /*! step3: output the result (TODO configure output rate)*/
        if (rgioe_get_status(rgioe_dev) == RGIOE_STATUS_NAVIGATION) {
            rgioe_get_result(rgioe_dev, &result);
            writer.Write(result);
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
    (void *) argv0;
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

#if 0
int main1(int argc, char *argv[]) {
    logInit(argv[0], "./log/");
    ShowFusionConfig();
#if ENABLE_FUSION_RECORDER
    Recorder::GetInstance().Initialize(argv[0]);
#endif
    if (argc < 2) {
        LOG(INFO) << CopyRight << endl;
        return 1;
    }
    Config config;
    config.LoadFrom(argv[1]);
    bool ok;
    string error_msg;
    RgioeImuData imu;
    RgioeGnssData gnss;
    NavOutput out;
    RgioeOdometerData odometer;
    PressureData press;
    ok = config.LoadImuPara(error_msg);
    RgioeOption opt = config.GetOption();
    LOG_IF(FATAL, !ok) << "IMU parameter file read failed:" << error_msg;
    LOG(INFO) << config.ToStdString();
    LOG(INFO) << opt.imuPara;

    std::shared_ptr<ReaderBase<RgioeImuData>> imu_reader;
    std::shared_ptr<ReaderBase<RgioeGnssData>> gnss_reader;
    std::shared_ptr<ReaderBase<RgioeOdometerData>> odometer_reader;
    std::shared_ptr<ReaderBase<PressureData>> press_reader;

    imu_reader = std::make_shared<IMUReader>(config.imu_config.file_path, config.imu_config.format,
                                             config.imu_config.frame, false, config.imu_config.d_rate);
    if (!imu_reader->IsOk()) {
        LOG(FATAL) << fmt::format("IMU file({}) not found", config.imu_config.file_path);
        return -1;
    }
    LOG(INFO) << fmt::format("start time {:.5}", config.start_time);
    imu_reader->ReadUntil(config.start_time, &imu);

    if (config.gnss_config.enable) {
        gnss_reader = std::make_shared<GnssReader>(config.gnss_config.file_path, config.gnss_config.format);
        if (!gnss_reader->IsOk()) {
            LOG(WARNING) << fmt::format("Gnss file({}) not found", config.gnss_config.file_path);
        }
        gnss_reader->ReadUntil(imu.gpst, &gnss);
        LOG_IF(ERROR, !gnss_reader->IsOk()) << "GNSS file not reach " << imu.gpst << " but reach " << gnss.gpst;
    }
    if (config.odometer_config.enable) {
        odometer_reader = std::make_shared<OdometerReader>(config.odometer_config.file_path);
        if (!odometer_reader->IsOk()) {
            LOG(WARNING) << fmt::format("Odometer file({}) not found", config.gnss_config.file_path);
        }
        odometer_reader->ReadUntil(imu.gpst, &odometer);
    }
    if (config.pressure_config.enable) {
        press_reader = std::make_shared<BmpReader>(config.pressure_config.file_path);
        if (!odometer_reader->IsOk()) {
            LOG(WARNING) << fmt::format("Press file({}) not found", config.pressure_config.file_path);
        }
        press_reader->ReadUntil(imu.gpst, &press);
    }
    Outage outage_cfg{config.outage_config.start, config.outage_config.stop, config.outage_config.outage,
                      config.outage_config.step};// = cfg.outage_config();
    NavWriter writer(config.output_config.file_path, config.output_config.format);

    NavEpoch nav;
    if (opt.align_mode == RgioeAlignMode::ALIGN_MOVING) {
        LOG(INFO) << "Align moving mode, wait for GNSS";
        AlignMoving align{opt};
        do {
            imu_reader->ReadNext(imu);
            align.Update(imu);
            if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
                auto v = align.Update(gnss);
                LOG(INFO) << "algn: " << gnss.gpst << "\tvel=" << v << "m/s";
                if (!gnss_reader->ReadNext(gnss)) {
                    LOG(ERROR) << "GNSS read finished,but align not complete!";
                    return 0;
                }
            }
        } while (!align.alignFinished() and imu_reader->IsOk());
        if (!align.alignFinished()) {
            LOG(ERROR) << "GNSS read finished,but align not complete!";
            return 0;
        }
        nav = align.getNavEpoch();
    } else if (opt.align_mode == ALIGN_USE_GIVEN) {
        auto nav_ = config.align_config.init_pva;
        nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
    } else {
        LOG(ERROR) << "supported align mode" << (int) opt.align_mode;
        return 1;
    }

    Timer timer;
/*第一步：初始化*/
    DataFusion df;
    df.Initialize(nav, opt);
    LOG(INFO) << "initial position:" << df.Output();

    if (config.pressure_config.enable) press_reader->ReadUntil(imu.gpst, &press);
    if (config.gnss_config.enable) gnss_reader->ReadUntil(imu.gpst, &gnss);
    if (config.odometer_config.enable) odometer_reader->ReadUntil(imu.gpst, &odometer);
/* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
    LOG(INFO) << "start:" << imu.gpst << ",end:" << config.stop_time;
    while (((config.stop_time <= 0) || (config.start_time > 0 && imu.gpst < config.stop_time)) && imu_reader->IsOk()) {
        if (!imu_reader->ReadNext(imu))break;
        /*第二步 时间更新*/
        df.TimeUpdate(imu);
        /* GNSS更新 */
        if (config.gnss_config.enable and gnss_reader->IsOk() and fabs(gnss.gpst - imu.gpst) < 0.6 / opt.d_rate) {
            if (config.outage_config.enable and outage_cfg.IsOutage(gnss.gpst)) {
                gnss.mode = GnssMode::INVALID;/*手动设置GNSS模式为INVALID*/
            }
            df.MeasureUpdatePos(gnss);
            do {
                if (!gnss_reader->ReadNext(gnss)) {
                    LOG(WARNING) << "gnss read failed";
                }
            } while (gnss_reader->IsOk() && gnss.gpst < imu.gpst);
        }
        /*里程计更新*/
        if (opt.odo_enable and odometer_reader->IsOk() and fabs(odometer.gpst - imu.gpst) < 1.2 / opt.d_rate) {
            df.MeasureUpdateVel(odometer.forward);
            Vec3d angle = df.estimator_.GetEulerAngles().transpose();
            do {
                if (!odometer_reader->ReadNext(odometer)) {
                    LOG(WARNING) << "vel read failed" << odometer.gpst;
                }
            } while (odometer.gpst < imu.gpst);
        }
        if (config.pressure_config.enable and fabs(press.gpst - imu.gpst) < 1.0 / opt.d_rate) {
            double height = 44330 * (1 - pow(press.pressure / 101325.0, 0.19));
            double z = df.MeasureUpdateRelativeHeight(height);
            LOG_EVERY_N(INFO, 100 * opt.d_rate) << "Pressure update:" << gnss << "at " << imu.gpst;
            press_reader->ReadNext(press);
        }
        if (!opt.enable_rts) {
            writer.update(df.Output());
        }
    }
    if (opt.enable_rts) {
        /* RTS模式下,输出结果顺序是反的,因此用栈结构存储 */
        LOG(INFO) << "Start RTS smooth,final state is " << df.Output();
//        LOG(INFO) << "State vector is " << df.Xd.transpose();
        bool finished;
        std::list<NavOutput> result;
        do {
            finished = df.RtsUpdate();
            out = df.Output();
            result.push_back(out);
        } while (!finished);
        LOG(INFO) << "Saving result...";
        while (!result.empty()) {
            writer.update(result.back());
            result.pop_back();
        }
        LOG(INFO) << "RTS smooth finished\n";
    }
    LOG(INFO) << "Process finished";
    /*show summary and reports:*/
    LOG(INFO) << "\n\tSummary:\n"
              << "\tAll epochs:" << df.EpochCounter() << '\n'
              << "\tTime for Computing:" << static_cast<double>(timer.elapsed()) / 1000.0 << "s" << '\n'
              << "\tFinal PVA:" << df.Output() << '\n';
    LOG_IF(INFO, config.outage_config.enable)
                    << "outage:" << config.outage_config.outage << " s, from " << config.outage_config.start << " to "
                    << config.outage_config.stop;
    LOG(INFO) << "\tThe result is saved to " << config.output_config.file_path;
#if RGIOE_ESTIMATE_GNSS_LEVEL_ARM == 1
    LOG(INFO) << "Final lever arm:" << df.lb_gnss.transpose();
#endif
#if ENABLE_FUSION_RECORDER
    LOG(INFO) << "\tRecorder was saved to " << Recorder::GetInstance().GetRcdFilename();
#endif
    ERROR_EXIT:
    writer.stop();
    return 0;
}
#endif