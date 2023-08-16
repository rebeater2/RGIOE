/**
* @file: LooselyCouple2020_cpp nav_struct.cpp
* @author: rebeater
* @function: 存放全局结构体, C style
* @date: 2020/11/10
* @version: 1.0.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H
#define LOOSELYCOUPLE2020_CPP_NAV_STRUCT_H

#define _PI ((RgioeFloatType)3.1415926535897932)
#define _hour ((RgioeFloatType)3600)
#define _minute ((RgioeFloatType)60)
#define _sqrt_h ((RgioeFloatType)60)
#define _deg ((RgioeFloatType)(_PI/180.0))
#define _mGal ((RgioeFloatType)(1e-5))
#define _ppm ((RgioeFloatType)(1e-6))
#define _knot ((RgioeFloatType)(0.5144444))
#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif
#include "RgioeDefine.h"

typedef struct {
  float acce_bias[3];
  float gyro_bias[3];
} Bias;
typedef enum  {
  SENSOR_NULL = 0x0U,
  SENSOR_IMU = 0x01U,
  SENSOR_GNSS = 0x02U,
  SENSOR_ODO = 0x04U,
  SENSOR_ZUPT = 0x08U,
  SENSOR_NHC = 0x10U,
  SENSOR_CAMERA = 0x20U,
  SENSOR_LIDAR = 0x40U,
  SENSOR_HEIGHT = 0x80U,
}RgioeSensorType;
typedef enum {
  ALIGN_USE_GIVEN = 2,
  ALIGN_MOVING = 0,
  ALIGN_STATIONARY = 1
} RgioeAlignMode;
typedef struct {
  double gpst;
  double pressure;
}PressureData;

typedef struct {
  double gpst;
  double gyro[3];
  double acce[3];
} RgioeImuData;

typedef enum {
  INITIAL = 0,
  SPP = 1,
  RTK_DGPS = 2,
  UNKNOWN_PPS = 3,
  RTK_FIX = 4,
  RTK_FLOAT = 5,
  INVALID = 6,
  SBAS,
  PPP
} GnssMode;

typedef enum {
  SYSTEM_START,
  SYSTEM_WAIT_FOR_TIME,
  SYSTEM_ALIGNING,
  SYSTEM_NAVIGATION,
  SYSTEM_ERROR,
} SystemState;

typedef struct {
  double lat;
  double lon;
  float height; /*32*/
  float pos_std[3];
#if REAL_TIME_MODE == 0
  float vn[3];
  float vn_std[3];
  double gpst;
  float hdop;
  float pdop;
  float gdop;
  float yaw;/*0~360,-1表示无效*/
  float yaw_std;/*32 + 4*6 = 56 */
  float pitch;
  float pitch_std;
#endif
  int week;/*4*/
  int ns; /*大于128不可能！！！*/
  int mode; /*  和 NMEA的模式定义保持一致*/
} RgioeGnssData;/*64 bytes*/

typedef struct {
  float arw;
  float vrw;
  float ab_ini[3];
  float gb_ini[3];
  float as_ini[3];
  float gs_ini[3];
  float ab_std[3];
  float gb_std[3];
  float gs_std[3];
  float as_std[3];
  float at_corr;
  float gt_corr;
} ImuPara;
typedef struct {
  unsigned short sensors;
  unsigned short gnss_mode;
} NavInfo;
typedef struct {
  int week;									/*GPS week*/
  double gpst;								/*GPS TOW*/
  double lat;								/*Latitude*/
  double lon;								/*Longitude*/
  float height;								/*Height*/
  float pos_std[3];							/*Position Standard deviation*/
  float vn[3];								/*Velocity in NED*/
  float vn_std[3];							/*Velocity Standard deviation*/
  float atti[3];							/*Attitude in NED, roll pitch heading,Unit deg*/
  float atti_std[3];						/*Attitude Standard deviation*/
  NavInfo info;								/*Navigation Information*/
  float gb[3];								/*gyroscope bias*/
  float ab[3];								/*accelerator bias*/
  float gs[3];								/*gyroscope scale factor*/
  float as[3];								/*accelerator scale factor*/
  float kd;									/*Odometer scale factor*/
} NavOutput;
/*sizeof(NavOutput)=128*/
typedef struct {
  double latitude;
  double longitude;
} LatLon;
typedef struct {
  double lat;
  double lon;
  float vn[3];
  float atti[3];
  NavInfo info;
  float height;
} NavPva;
/*typedef enum {
  UseGiven = 0,
  Moving = 1,
  Static = 2
} RgioeAlignMode;*/

typedef struct {
  ImuPara imuPara;
  int d_rate;
  int align_mode;
  float align_vel_threshold;
  int enable_gnss;
  float lb_gnss[3];
  float gnss_std_scale;
  int nhc_enable;
  float nhc_std[2];
  int zupt_enable;
  float zupt_std;
  int zupta_enable;
  float zupta_std;
  int odo_enable;
  float odo_std;
  float odo_scale;
  float odo_scale_std;
  float lb_wheel[3];
  float angle_bv[3];
  float pos_std[3];
  float vel_std[3];
  float atti_std[3];
  int output_project_enable;
  float pos_project[3];
  float atti_project[3];
  int enable_rts;
} RgioeOption;

typedef struct {
  unsigned char year;
  unsigned char mon;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char second;
} DateTime;
typedef struct {
  float forward;
  float angular;
  double gpst;
} Velocity;
typedef enum {
  IMU_FRAME_RFU = 1,
  IMU_FRAME_FRD = 0
} IMUFrame;

/**
 * CameraInfo from Uart
 */
typedef struct {
  unsigned int Header; /*4*/
  char ImageName[24]; /*24*/
  float Vn[3];/*4*3 = 12*/
  float VnStd[3];/*12*/
  unsigned int crc;/*4*/
} CameraInfo;

typedef struct {
  CameraInfo camera_info_;
  double time_s;
} CameraEvent;

typedef struct{
  double gpst;
  double velocity;
  double angular;
}AuxiliaryData;

#endif //LOOSELY_COUPLE_H7_NAV_STRUCT_H
