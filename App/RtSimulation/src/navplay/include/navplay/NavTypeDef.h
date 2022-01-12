/**
* @file NavTypeDef.h in Project
* @author rebeater
* @comment
* Create on 1/10/22 9:54 AM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_

#define _PI 3.1415926535897932
#define _hour 3600
#define _minute 60
#define _sqrt_h 60
#define _deg (_PI/180.0)
#define _mGal (1e-5)
#define _ppm (1e-6)

#ifdef __cplusplus
#include <stdint.h>
#else
#include <stdint.h>
#endif
//#include "Define.h"

typedef struct {
  float acce_bias[3];
  float gyro_bias[3];
} Bias;

typedef enum {
  SENSOR_NULL = 0x0U,
  SENSOR_IMU = 0x01U,
  SENSOR_GNSS = 0x02U,
  SENSOR_ODO = 0x04U,
  SENSOR_ZUPT = 0x08U,
  SENSOR_NHC = 0x10U,
  SENSOR_CAMERA = 0x20U,
  SENSOR_LIDAR = 0x40U,
  } SensorType;
typedef enum {
  ALIGN_USE_GIVEN = 0,
  ALIGN_MOVING = 1,
  ALIGN_STATIONARY = 2
} AlignMode;

typedef struct {
  double gpst;
  double gyro[3];
  double acce[3];
} ImuData;

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

//#pragma pack(4)
typedef struct {
  double lat;
  double lon;
  float height; /*32*/
  float pos_std[3];
  float yaw;/*0~360,-1表示无效*/
  float pitch;
  short pitch_std_100;
  short yaw_std_100;/*32 + 4*6 = 56 */
  short week;/*4*/
  unsigned  char ns; /*大于128不可能！！！*/
  unsigned char mode; /*  和 NMEA的模式定义保持一致*/
  //  unsigned char reserved[24];
} GnssData;/*64 bytes*/
typedef struct {
  GnssData data;
  double gpst;
}GnssDataMsg;

//#pragma unpack()
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

typedef struct {
  unsigned short sensors;
  unsigned short mode;
} NavInfo;

typedef struct {
  double lat;
  double lon;
  float vn[3];
  float atti[3];
  NavInfo info;
  float height;
} NavOutput;

/*sizeof(NavOutput)=128*/
typedef struct {
  double latitude;
  double longitude;
} LatLon;
typedef struct {
  double gpst;
  double lat;
  double lon;
  float h;
  float vn;
  float ve;
  float vd;
  float roll;
  float pitch;
  float yaw;
  float ab[3];
  float gb[3];
  NavInfo info;
  /*    double pos_std_[3];
	  double vel_std_[3];
	  double atti_std_[3];*/
  long mode;
} NavPva;
/*typedef enum {
  UseGiven = 0,
  Moving = 1,
  Static = 2
} AlignMode;*/

typedef struct {
  ImuPara imuPara;
  NavPva init_epoch;

  int d_rate;
  AlignMode align_mode;
  uint8_t nhc_enable;
  uint8_t zupt_enable;
  uint8_t zupta_enable;
  uint8_t outage_enable;
  uint8_t odo_enable;
  float nhc_var;
  float zupt_var;
  float zupta_var;
  float lb_gnss[3];
  float odo_var;
  float odo_wheel_radius;
  float lb_wheel[3];
  float angle_bv[3];
  float pos_std[3];
  float vel_std[3];
  float atti_std[3];
  float nhc_std[2];
#if KD_IN_KALMAN_FILTER == 1
  float  kd_init;
  float  kd_std ;
#endif
} Option;

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

typedef struct {
  double gpst;
  double velocity;
  double angular;
}	AuxiliaryData;
typedef enum {
  DATA_TYPE_IMU = 0,
  DATA_TYPE_GNSS = 1,
  DATA_TYPE_VEL = 2,
  DATA_TYPE_BMP = 3,
  DATA_TYPE_RST = 4
} DataTypeDef;

typedef struct {
  int16_t temp_low;
  int16_t temp_out;
  int16_t x_gyro_low;
  int16_t x_gyro_out;

  int16_t y_gyro_low;
  int16_t y_gyro_out;
  int16_t z_gyro_low;
  int16_t z_gyro_out;

  int16_t x_acce_low;
  int16_t x_acce_out;
  int16_t y_acce_low;
  int16_t y_acce_out;

  int16_t z_acce_low;
  int16_t z_acce_out;

  uint32_t id;
  //  uint16_t reserved;
}ImuRawKy;

typedef enum {
  IMU_FILE_IMD = 1,
  IMU_FILE_IMUTXT = 0
} IMUFileFormat;
typedef enum {
  IMU_FRAME_RFU = 1,
  IMU_FRAME_FRD = 0,
  } IMUFrame;

typedef struct {
  union {
    struct{
      ImuRawKy raw_;
      short ab_mGal[3];
      short gb_degph[3];
    } imu_;
    NavOutput rst_;
    GnssData gnss_;
    Velocity vel_;
  };
  double gpst;
  DataTypeDef type_;
  uint8_t reserved[7];
} RawDataDef;

#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_
