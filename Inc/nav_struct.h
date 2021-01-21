/**
* @file: LooselyCouple2020_cpp nav_struct.cpp
* @author: rebeater
* @function: 存放全局结构体
* @date: 2020/11/10
* @version: 1.0.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
#define LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
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
    double pdop;
    double vdop;
    int ns;
    int mode;
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
typedef struct {
    int d_rate;
    float lb_gnss[3];/*f r d*/
    double pos_std[3];
    double vel_std[3];
    double atti_std[3];
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

#endif //LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
