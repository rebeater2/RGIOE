/**
* @file rgioe.h in InsCubeBsp
* @author linfe
* @comment
* Create on 2022/12/22 9:22
* @version 1.0
**/

#ifndef INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_H_
#define INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_H_


#ifdef __cplusplus
extern "C" {
#endif
/*嵌入式实时模式*/
#ifndef REAL_TIME_MODE
#define REAL_TIME_MODE 1
#endif

#include "RgioeConfig.h"
#include "RgioeDataType.h"

#define RGIOE_DATA_BUFFER_SIZE  1024

typedef enum {
  RGIOE_OK = 0,
  RGIOE_NULL_INPUT = 1,
  RGIOE_IN_INITIALIZE = 2,
  RGIOE_UNIMPLEMENTED = 0XF,
} rgioe_error_t;



rgioe_error_t rgioe_init(void *rgioe_dev,const RgioeOption *opt);

rgioe_error_t rgioe_timeupdate(void *rgioe_dev,double timestamp,RgioeImuData * imu_inc);

rgioe_error_t rgioe_gnssupdate(void *rgioe_dev,double timestamp,RgioeGnssData *gnss);

rgioe_error_t rgioe_get_atti(void *rgioe_dev,float atti[3], float *std);

rgioe_error_t rgioe_get_pos(void *rgioe_dev,double pos[3], float *std);

rgioe_error_t rgioe_get_vel(void *rgioe_dev,float vel[3],float *std);

#ifdef __cplusplus
};
#endif
#endif //INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_H_
