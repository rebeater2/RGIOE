//
// Created by rebeater on 2020/12/17.
//
#include <fstream>
#include <FileIO.h>
#include <Config.h>
#include "DataFusion.h"
#include "NavLog.h"
#include "NavStruct.h"
#include <Timer.h>
#include <Alignment.h>

#define MULTI_THREAD 1


int main(int argc, char *argv[]) {
    std::string help_msg = "usage: dataFusion configure.yml";
    logInit(argv[0], "../log/");
    if (argc < 2) {
        cout << help_msg << endl;
        return 1;
    }
    Config cfg(argv[1]);
    logi << "imu path:" << cfg.imu_filepath;
    Option opt = cfg.getOption();

    /*Use Given Initial State*/

    ImuData imu;
    GnssData gnss;
    NavOutput out;
/*移动文件指针到指定的开始时间*/
    ifstream f_imu(cfg.imu_filepath);
    do { readImu(f_imu, &imu, opt.imu_format); } while (imu.gpst < cfg.start_time and f_imu.good());
    ifstream f_gnss(cfg.gnss_filepath);
    string buffer;
    /*跳过GPS文件的开头4行*/
    for (int i = 0; i < 4; i++) getline(f_gnss, buffer);
    do {
        readGnss(f_gnss, &gnss, opt.gnss_format);
    } while (gnss.gpst < cfg.start_time and f_gnss.good());
    readGnss(f_gnss, &gnss, opt.gnss_format);
#if MULTI_THREAD == 1
    NavWriter writer(cfg.output_filepath);
#else
    ofstream f_nav(cfg.output_filepath);
#endif

    /*初始对准*/
    NavEpoch nav;
    if (opt.alignmode == ALIGN_MOVING) {
        AlignMoving align{7,opt};
        do {
            readImu(f_imu, &imu, opt.imu_format);
            align.Update(imu);
            if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
                align.Update(gnss);
                readGnss(f_gnss, &gnss, opt.gnss_format);
            }
        } while (!align.alignFinished() and f_imu.good() and f_gnss.good());
        if (!align.alignFinished()) {
            loge << "align failed";
            return -1;
        }
        nav = align.getNavEpoch();
    } else if (opt.alignmode == ALIGN_USE_GIVEN) {
        auto nav_ = cfg.getInitNav();
        nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
    }
    Timer timer;
    DataFusion::Instance().Initialize(nav, opt);
    /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
    while ((cfg.end_time <= 0) || (cfg.end_time > 0 && imu.gpst < cfg.end_time)) {
        readImu(f_imu, &imu, opt.imu_format);
        if (!f_imu.good())break;
        DataFusion::Instance().TimeUpdate(imu);
        if (fabs(gnss.gpst - imu.gpst) < 1.0 / opt.d_rate) {
            if (DataFusion::Instance().MeasureUpdatePos(gnss) < 0) {
                LOG_EVERY_N(INFO, 1) << "in outage mode" << gnss.gpst;
            }
            LOG_EVERY_N(INFO, 100) << "measure update: " << gnss.gpst << " imu.gpst " << imu.gpst;
            readGnss(f_gnss, &gnss, opt.gnss_format);
            if (!f_gnss.good())break;
        }
#if MULTI_THREAD == 1
        out = DataFusion::Instance().Output();
        writer.update(out);
#else
        f_nav << DataFusion::Instance().Output() << '\n';
#endif
    }
#if MULTI_THREAD == 1
    logi << "resolve finished, waiting for writing file";
    logi << "time used:" << timer.elapsed() / 1000.0 << "s";
    writer.stop();
    logi << "write finished\n";
#else
    f_nav.close();
#endif
    f_imu.close();
    f_gnss.close();
    logi << "time used:" << timer.elapsed() / 1000.0 << "s";
    logi << DataFusion::Instance().Output() << endl;
    return 0;
}

