/**
* @file rgioe_base_type.h in InsCubeBsp
* @author linfe
* @comment
* Create on 2022/12/23 9:38
* @version 1.0
**/

#ifndef INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_BASE_TYPE_H_
#define INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_BASE_TYPE_H_

typedef struct {
  double stimestamp;        /* current timestamp*/
  float deltat;             /* delta t*/

} rgioe_data_t;

typedef enum {
  rgioe_ok = 0,
} rgioe_error_t;

typedef struct {
  double timestamp;
  float acce[3];
  float gyro[3];
}rgioe_imu_inc_t;

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

typedef struct {
  double lat;
  double lon;
  float height; /*32*/
  float pos_std[3];
  int week;/*4*/
  int ns; /*大于128不可能！！！*/
  int mode; /*  和 NMEA的模式定义保持一致*/
}rgioe_gnss_t;





#endif //INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_BASE_TYPE_H_
