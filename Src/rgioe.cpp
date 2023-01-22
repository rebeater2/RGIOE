/**
* @file rgioe.cpp in InsCubeBsp
* @author linfe
* @comment
* Create on 2022/12/22 9:22
* @version 1.0
**/
#include "rgioe.h"
#include <DataFusion.h>
#include <Alignment.h>

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
NavOutput init_pva{0, 0, 0, 0};
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
typedef struct {
  DataFusion df;
  AlignMoving am;
  float time_delay;
} RgioeData_t;

/**
 * <h2> Initialize function
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param opt necessary options
 * @return Error Code as defined by rgioe_error_t
 */
rgioe_error_t rgioe_init(void *rgioe_dev, const RgioeOption *opt) {
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    rd->df = DataFusion();
    if (opt) {
        rd->df.opt = *opt;
    } else {
#ifdef ENABLE_DEFAULT_OPTION
        rd->df.opt = default_option;
#else
        return RGIOE_NULL_INPUT;
#endif
    }
    rd->am = AlignMoving(*opt);
    return RGIOE_OK;
}

/**
 * <h2> Time update function
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param timestamp current timestamp for estimate time-delay
* @param imu_inc IMU data in increment format, acce: m/s, gyro: rad
* @return Error Code as defined by rgioe_error_t
*/
rgioe_error_t rgioe_timeupdate(void *rgioe_dev, double timestamp, RgioeImuData *imu_inc) {
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    if (imu_inc == nullptr) {
        return RGIOE_NULL_INPUT;
    }
    rd->time_delay = (float)(timestamp - imu_inc->gpst);

    /* align and initialize */
    if (!rd->am.alignFinished()) {
        rd->am.Update(*imu_inc);
        if (rd->am.alignFinished()) {
            rd->df.Initialize(rd->am.getNavEpoch(), rd->df.opt);
        }
        return RGIOE_IN_INITIALIZE;
    }
    /* integrate by IMU if alignment is finished*/
    rd->df.TimeUpdate(*imu_inc);
    return RGIOE_OK;
}

/**
 * <h2> observation update for GNSS position data
 * @param rgioe_dev pointer to RgioeData_t to get access to global variables
 * @param timestamp current timestamp for estimate time-delay
 * @param gnss pointer to GNSS position data
 * @return Error Code as defined by rgioe_error_t
 */
rgioe_error_t rgioe_gnssupdate(void *rgioe_dev, double timestamp, RgioeGnssData *gnss) {
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    if (!gnss) {
        return RGIOE_NULL_INPUT;
    }
    rd->df.MeasureUpdatePos(*gnss);
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
rgioe_error_t rgioe_get_atti(void *rgioe_dev,float atti[3], float *std){
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    NavOutput result = rd->df.Output();
    for (int i = 0; i < 3; ++i){
        atti[i] = result.atti[i];
    }
    if (std){
        for (int i = 0; i < 3; ++i) {
            std[i] = result.atti_std[i];
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
rgioe_error_t rgioe_get_pos(void *rgioe_dev,double pos[3], float *std){
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    NavOutput result = rd->df.Output();
    pos[0] = result.lat;
    pos[1] = result.lon;
    pos[2] = result.height;
    if (std){
        for (int i = 0; i < 3; ++i) {
            std[i] = result.atti_std[i];
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
rgioe_error_t rgioe_get_vel(void *rgioe_dev,float vel[3],float *std){
    auto rd = static_cast<RgioeData_t *>(rgioe_dev);
    NavOutput result = rd->df.Output();
    for (int i = 0; i < 3; ++i){
        vel[i] = result.vn[i];
    }
    if (std){
        for (int i = 0; i < 3; ++i) {
            std[i] = result.vn_std[i];
        }
    }
    return RGIOE_OK;
}