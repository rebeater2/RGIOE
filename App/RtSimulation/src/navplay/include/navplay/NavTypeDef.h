/**
* @file NavTypeDef.h in Project
* @author rebeater
* @comment
* Create on 1/10/22 9:54 AM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_
#include "NavStruct.h"
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
}GnssRawDef;

typedef enum {
  DATA_TYPE_IMU = 0,
  DATA_TYPE_GNSS = 1,
  DATA_TYPE_VEL = 2,
  DATA_TYPE_BMP = 3,
  DATA_TYPE_RST = 4
} DataTypeDef;

typedef struct {
  unsigned char  vh_;
  unsigned char vl_;
  unsigned char ah_;
  unsigned char al_;
  int reserved;
} VelocityRawDef;

/*结构体接口，务必保证这个结构体的体积是64 bytes*/
typedef struct {
  union {
    struct{
      ImuRawKy raw_;
      short ab_mGal[3];
      short gb_degph[3];
    } imu_;
    NavPva rst_;
    GnssRawDef gnss_;
    VelocityRawDef vel_;
  };
  double gpst;
  DataTypeDef type_;
  uint8_t reserved[7];
} RawDataDef;

#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_NAVSTRUCT_H_
