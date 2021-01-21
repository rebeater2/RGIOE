//
// Created by rebeater on 2020/12/17.
//
#include <fstream>
#include <f_io.h>
#include <Config.h>
#include "DataFusion.h"
#include "navigation_log.h"
#include "nav_struct.h"




int main(int argc, char *argv[]) {
    std::string help_msg = "usage: dataFusion configure.yml";
    logInit(argv[0], "../log/");
    if (argc < 2) {
        cout << help_msg << endl;
        return 1;
    }
    Config cfg(argv[1]);
    logi <<"imu path:"<< cfg.imu_filepath;
    Option opt = cfg.getOption();
    logi <<"gnss path"<< opt.lb_gnss[0];
    auto nav_ = cfg.getInitNav();
    auto nav = makeNavEpoch(nav_, opt);/* TODO 目前没有实现动态对准 */
    ImuData imu;
    GnssData gnss;
    DataFusion df(nav, opt);
    logi << opt.imuPara;
    logi << "init nav:" << nav;


    ifstream f_imu(cfg.imu_filepath);
    do { f_imu >> imu; } while (imu.gpst < cfg.start_time and f_imu.good());

    ifstream f_gnss(cfg.gnss_filepath);
    do { f_gnss >> gnss; } while (gnss.gpst < cfg.start_time and f_gnss.good());
    f_gnss >> gnss;

    ofstream f_nav(cfg.output_filepath);

    while (true) {
        f_imu >> imu;
        if (!f_imu.good())break;
        df.TimeUpdate(imu);
        if (fabs(gnss.gpst - imu.gpst) < 1.0 / cfg.d_rate) {
            df.MeasureUpdatePos(gnss);
            LOG_EVERY_N(INFO, 100) << "measure update: " << gnss.gpst << " imu.gpst " << imu.gpst;
            f_gnss >> gnss;
            if (!f_gnss.good())break;
        }
        f_nav << df.Output() << endl;
    }
    logi << df.Output() << endl;

    return 0;
}

