/**
* @file: LooselyCouple2020_cpp nav_struct.cpp
* @author: rebeater
* @function: 存放全局结构体
* @date: 2020/11/10
* @version: 1.0.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
#define LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H

#include <stdint.h>

#define _PI 3.1415926535897932
#define _hour 3600
#define _minute 60
#define _sqrt_h 60
#define _deg (_PI/180.0)
#define _mGal (1e-5)
#define _ppm (1e-6)

//const double pi = 3.1415926535897932384626;
//const double deg = pi / 180.0;
//const double hour = 3600;
//const double minute = 60;

//#define Vec3d Eigen::Vector3d
typedef struct {
    double gpst;
    double gyro[3];
    double acce[3];
} ImuData;
typedef struct {
    double gpst;
    double lat;
    double lon;
    double height;
    double pos_std[3];
    double hdop;
    int ns;
    int mode;
    int week;
} GnssData;
typedef struct {
    double arw;
    double vrw;
    double ab_ini[3];
    double gb_ini[3];
    double as_ini[3];
    double gs_ini[3];
    double ab_std[3];
    double gb_std[3];
    double gs_std[3];
    double as_std[3];
    double at_corr;
    double gt_corr;
} ImuPara;
typedef enum {
    IMU_FORMAT_IMD = 0,
    IMU_FORMAT_IMUTXT = 1
} ImuFileFormat;
typedef enum {
    GNSS_TXT_POS_7 = 0,
    GNSS_BIN_POS_7 = 1,
    GNSS_TXT_POS_14 = 2,
    GNSS_BIN_POS_14 = 3,
    RTKLIB_TXT_POS = 4,
    GNSS_TXT_POS_VEL = 5,
    GNSS_TXT_GGA = 6,
    RESERVED = 6,
} GnssFileFormat;
typedef  enum  {
    ALIGN_USE_GIVEN = 0,
    ALIGN_MOVING = 1,
    ALIGN_STATIONARY = 2
}AlignMode;
typedef struct {
    int d_rate;
    ImuFileFormat imu_format;
    GnssFileFormat gnss_format;
    AlignMode alignmode;
    uint8_t nhc_enable;
    uint8_t zupt_enable;
    uint8_t zupta_enable;
    uint8_t outage_enable;
    int outage_start;
    int outage_stop;
    int outage_time;
    int outage_step;
    float lb_gnss[3];/*f r d*/
    float lb_wheel[3];
    double pos_std[3];
    double vel_std[3];
    double atti_std[3];
    double angle_bv[3];/*安装角*/
    double nhc_std[2];

    ImuPara imuPara;
} Option;
typedef struct {
    double gpst;
    double pos[3];
    double vn[3];
    double atti[3];
    double gb[3];
    double ab[3];

} NavOutput;

typedef struct {
    double latitude;
    double longitude;
} LatLon;
typedef struct {
    float acce_bias[3];
    float gyro_bias[3];
}Bias;
#endif //LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
