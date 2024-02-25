/**
* @file rgioe.cpp in InsCubeBsp
* @author linfe
* @comment
* Create on 2022/12/22 9:22
* @version 1.0
**/
#include "rgioe.h"
#include "DataFusion.h"
#include "Alignment.h"

// 使用glog输出日志
#include <glog/logging.h>
extern int rgioe_log_impl(const char *fun,int line, const char *format, ...);
RGIOE_WEAK_FUNC int rgioe_log_impl(const char *fun,int line, const char *format, ...){
    RGIOE_UNUSED_PARA(fun);
    RGIOE_UNUSED_PARA(line);
    RGIOE_UNUSED_PARA(format);
    return 0;
}

#ifndef __FILE_NAME__
#define LOG_INFO(...) rgioe_log_impl(__FILE__,__LINE__,__VA_ARGS__)
#else
#define LOG_INFO(...) rgioe_log_impl(__FILE_NAME__,__LINE__,__VA_ARGS__)
#endif

const char *rgioe_build_info = "build on " __DATE__ " " __TIME__;
char CopyRight[] = "GNSS/INS/ODO Loosely-Coupled Program (1.01)\n"
                   "Copyright(c) 2019-2021, by Bao Linfeng, All rights reserved.\n";


#define ENABLE_DEFAULT_OPTION
#ifdef ENABLE_DEFAULT_OPTION
const ImuPara default_imupara{0.15 * _deg / _sqrt_h, 0.25 / _sqrt_h,
                              -0 * _mGal, 0 * _mGal, -0 * _mGal,
                              0 * _deg / _hour, -0 * _deg / _hour, 0 * _deg / _hour,
                              0, 0, 0,
                              0, 0, 0,
                              3.6 * _mGal, 3.6 * _mGal, 3.6 * _mGal,
                              3 * _deg / _hour, 3 * _deg / _hour, 3 * _deg / _hour,
                              1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
                              1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
                              1 * _hour, 1 * _hour
};

RgioeOption default_option{
        .imuPara=default_imupara,
        .d_rate = 125,
        .align_mode=RgioeAlignMode::ALIGN_USE_GIVEN,
        .align_vel_threshold = 1.4,
        .enable_gnss = 1,
        .lb_gnss={0, 0, 0},
        .gnss_std_scale = 1.0,
        .nhc_enable=1,
        .nhc_std= {0.1, 0.1},
        .zupt_enable=1,
        .zupt_std=0.01,
        .zupta_enable = 1,
        .zupta_std=0.01 * _deg,
        .odo_enable = 0,
        .odo_std = 0.1,
        .odo_scale = 1.14,
        .odo_scale_std  = 0,
        .lb_wheel={-0.317, -0.095, 0.03},
        .angle_bv={0, 0, 0},
        .pos_std={1, 1, 1},
        .vel_std={2, 2, 2},
        .atti_std={10 * _deg, 10 * _deg, 10 * _deg},
        .output_project_enable = 0,/*输出投影*/
        .pos_project = {0, 0, 0},/*投影到目标位置*/
        .atti_project = {0, 0, 0},/*投影到目标姿态*/
        .enable_rts = 0
};
#endif

/** <h2> global variables*/
struct RgioeData_t {
    DataFusion df;
    AlignMoving am;
    rgioe_status_t status{};
    RgioeOption opt{};
    rgioe_nav_pva_t cur_pva{};
};
const uint32_t rgioe_buffer_size = sizeof(RgioeData_t);

/**
 * <h2> Initialize function
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param opt pointer to necessary options
 * @return Error Code as defined by rgioe_error_t
 */
rgioe_error_t rgioe_init(uint8_t *rgioe_dev, const RgioeOption *opt, rgioe_nav_pva_t *init_nav) {
    auto rd = reinterpret_cast<RgioeData_t *>(rgioe_dev);
    rd->df = DataFusion();
    rd->am = AlignMoving();
    if (opt) {
        rd->opt = *opt;
    } else {
#ifdef ENABLE_DEFAULT_OPTION
        rd->df.opt = default_option;
#else
        return RGIOE_NULL_INPUT;
#endif
    }
    if (rd->opt.align_mode == ALIGN_USE_GIVEN and (init_nav == nullptr)){
        memset(&rd->cur_pva,0,sizeof(rd->cur_pva));
        LOG_INFO("Align mode is USE_GIVEN, initial navigation information should NOT be NULL");
        return RGIOE_FAULT_CONFIG;
    }
    if (init_nav) {
        rd->cur_pva = *init_nav;
    }
    rd->am.SetOption(*opt);
#if RGIOE_REALTIME_DEBUG == 1
    rd->trace("rgioe: AlignMoving created, return OK\n");
#endif
    rd->status = RGIOE_STATUS_INIT;
    return RGIOE_OK;
}

/**
 * <h2> Time update function
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param timestamp current timestamp for estimate time-delay
* @param imu_inc IMU data in increment format, acce: g*s, gyro: rad
* @return Error Code as defined by rgioe_error_t
*/
rgioe_error_t rgioe_timeupdate(uint8_t *rgioe_dev, double timestamp, const RgioeImuData *imu_inc) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    RGIOE_UNUSED_PARA(timestamp);
    if (imu_inc == nullptr) {
        return RGIOE_NULL_INPUT;
    }
    switch (rd->status) {
        case RGIOE_STATUS_INIT:
            if (rd->opt.align_mode == ALIGN_USE_GIVEN){
                auto nav = makeNavEpoch(rd->cur_pva,rd->opt);
                rd->df.Initialize(nav,rd->opt);
                rd->df.TimeUpdate(*imu_inc);
                rd->status = RGIOE_STATUS_NAVIGATION;
                LOG_INFO("UseGiven mode");
            }else{
                LOG_INFO("Start motion align");
                rd->status = RGIOE_STATUS_ALIGN;
            }
            break;
        case RGIOE_STATUS_ALIGN:
            rd->am.Update(*imu_inc);
            if (rd->am.alignFinished()) {
                auto nav = rd->am.nav;
                rd->df.Initialize(nav, rd->opt);
                rd->status = RGIOE_STATUS_NAVIGATION;
                LOG_INFO("align finished,status change to %d",rd->status);
            }
            break;
        case RGIOE_STATUS_NAVIGATION:
            rd->df.TimeUpdate(*imu_inc);
            break;
        default:
            break;
    }
    return RGIOE_OK;
}

/**
 * <h2> observation update for GNSS position data
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param timestamp current timestamp for estimate time-delay
 * @param gnss pointer to GNSS position data
 * @return Error Code as defined by rgioe_error_t
 */

rgioe_error_t rgioe_gnssupdate(uint8_t *rgioe_dev, double timestamp, const RgioeGnssData *gnss) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    RGIOE_UNUSED_PARA(timestamp);
    if (!gnss) {
        return RGIOE_NULL_INPUT;
    }
    switch (rd->status) {
        case RGIOE_STATUS_INIT:
            //rd->status = RGIOE_STATUS_ALIGN;
            break;
        case RGIOE_STATUS_ALIGN: {
            auto v  = rd->am.Update(*gnss);
            if (rd->am.alignFinished()) {
                LOG_INFO("motion align finished, current speed:%f", v);
            }
        }
            break;
        case RGIOE_STATUS_NAVIGATION:
            rd->df.MeasureUpdatePos(*gnss);
            break;
        default:
            break;
    }
    return RGIOE_OK;
}

/** <h2> functions to get result*/
/**
 * <h3> get attitude from rgioe
 * @param rgioe_dev
 * @param atti IMU attitude,in roll/pitch/yaw respected to Forward-Right-Down frame
 *             \n Unit: rad
 * @param std Attitude Std in rad
 * @return Error Code as defined by rgioe_error_t
 */
rgioe_error_t rgioe_get_atti(uint8_t *rgioe_dev, float atti[3], float *std) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    NavOutput result = rd->df.Output();
    for (int i = 0; i < 3; ++i) {
        atti[i] = (float)result.atti[i];
    }
    if (std) {
        for (int i = 0; i < 3; ++i) {
            std[i] = (float)result.atti_std[i];
        }
    }
    return RGIOE_OK;
}

/**
 * <h3> get position from rgioe
 * @param rgioe_dev
 * @param pos Position in latitude/longitude/height \n Unit:rad rad m
 * @param std
 * @return
   */
rgioe_error_t rgioe_get_pos(uint8_t *rgioe_dev, double pos[3], float *std) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    NavOutput result = rd->df.Output();
    pos[0] = result.lat;
    pos[1] = result.lon;
    pos[2] = result.height;
    if (std) {
        for (int i = 0; i < 3; ++i) {
            std[i] = (float)result.atti_std[i];
        }
    }
    return RGIOE_OK;
}

/**
 * <h3> get velocity from rgioe
 * @param rgioe_dev
 * @param vel velocity in North-East-Ground \n Unit:m/s
 * @param std m/s
 * @return
 */
rgioe_error_t rgioe_get_vel(uint8_t *rgioe_dev, float vel[3], float *std) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    NavOutput result = rd->df.Output();
    for (int i = 0; i < 3; ++i) {
        vel[i] = (float)result.vn[i];
    }
    if (std) {
        for (int i = 0; i < 3; ++i) {
            std[i] =(float)result.vn_std[i];
        }
    }
    return RGIOE_OK;
}

rgioe_status_t rgioe_get_status(uint8_t *rgioe_dev) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    return rd->status;
}

rgioe_error_t rgioe_get_result(uint8_t *rgioe_dev, rgioe_nav_pva_t *pva) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    *pva = rd->df.Output();
    return RGIOE_OK;
}


rgioe_error_t rgioe_deinit(uint8_t *rgioe_dev) {
    auto rd = (RgioeData_t *) (rgioe_dev);
    RGIOE_UNUSED_PARA(rd);
    return RGIOE_OK;
}