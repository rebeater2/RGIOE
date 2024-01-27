/**
* @file RawStruct.h in robot_decode
* @author rebeater
* @comment
* Create on 8/20/21 11:38 AM
* @version 1.0
**/

#ifndef ROBOT_DECODE__RAWSTRUCT_H_
#define ROBOT_DECODE__RAWSTRUCT_H_
#include "RgioeDataType.h"
namespace InsCube {
#include "stdint.h"
typedef struct {
  int32_t gyro[3]; /* 4 * 3*/
  int32_t acce[3]; /* 4 * 3*/
  double time_s;
} ImuRawStim;

typedef struct {
  uint16_t stat;/*STAT_OUT*/
  int16_t gyro[3];
  int16_t acce[3];
  int16_t temp;/*TEMP_OUT*/
  uint16_t cnt;/*SMPL_CNTR*/
  uint16_t checksum;
  uint32_t id;
} ImuRawAdi; /*size of ADI=2+2*3+2*3+2+2+2+4 =24 bytes*/
typedef struct {
  double gpst;
  uint32_t id;/*区分4个IMU数据*/
  uint32_t PIN_ID;
  int16_t acce[3];/* 2* 3*/
  int16_t temp;/* 2*/
  int16_t gyro[3];/*2 *3*/
  uint16_t RESERVED;
} ImuRawIcm;

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
} ImuRawKy;

typedef struct {
  unsigned char vh_;
  unsigned char vl_;
  unsigned char ah_;
  unsigned char al_;
  float kd;
  double gpst;
} VelocityRaw;
;
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
  unsigned char ns; /*大于128不可能！！！*/
  unsigned char mode; /*  和 NMEA的模式定义保持一致*/
  //  unsigned char reserved[24];
} GnssDataRaw;/*64 bytes*/

typedef struct{
  float pressure;
  float temperature;
}BmpRaw;
typedef enum {
  DATA_TYPE_IMU = 0,
  DATA_TYPE_GNSS = 1,
  DATA_TYPE_VEL = 2,
  DATA_TYPE_BMP = 3,
  DATA_TYPE_RST = 4
} DataTypeDef;
typedef struct {
  union {
	struct {
	  ImuRawAdi raw_;
	  short ab_mGal[3];
	  short gb_degph[3];
	} imu_;
	NavPva rst_;
	GnssDataRaw gnss_;
	VelocityRaw vel_;
	BmpRaw bmp_;
  };
  double gpst;
  DataTypeDef type_;
  uint8_t reserved[7];
} RawDataDef;
}
#endif //ROBOT_DECODE__RAWSTRUCT_H_
