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

#include "RgioeConfig.h"
#include "RgioeDataType.h"

#define RGIOE_REALTIME_DEBUG 0

extern const uint32_t rgioe_buffer_size;
typedef NavOutput rgioe_nav_pva_t;

extern const char *rgioe_build_info;
extern char CopyRight[];



typedef enum {
    RGIOE_OK = 0,
    RGIOE_NULL_INPUT = 1,
    RGIOE_IN_INITIALIZE = 2,
    RGIOE_UNIMPLEMENTED = 0XF,
} rgioe_error_t;

typedef enum {
    RGIOE_STATUS_INIT = 0,
    RGIOE_STATUS_ALIGN = 1,
    RGIOE_STATUS_NAVIGATION = 2,
    RGIOE_STATUS_ERROR = 3
} rgioe_status_t;
/**
 * initial rgioe fusion lib
 * @param rgioe_dev pointer to rgioe buffer, which should be malloc before and larger than rgioe_buffer_size
 * @param opt fusion options
 * @return RGIOE_OK
 */
rgioe_error_t rgioe_init(uint8_t *rgioe_dev, const RgioeOption *opt);

/**
 * time update function, which should be called with imu data
 * @param rgioe_dev
 * @param timestamp data timestamp
 * @param imu_inc imu data, FRD and inc data
 * @return
 */
rgioe_error_t rgioe_timeupdate(uint8_t *rgioe_dev, double timestamp, const RgioeImuData *imu_inc);

/**
 * GNSS measure update
 * @param rgioe_dev
 * @param timestamp
 * @param gnss
 * @return
 */
rgioe_error_t rgioe_gnssupdate(uint8_t *rgioe_dev, double timestamp, const RgioeGnssData *gnss);

/**
 * get fusion result
 * @param rgioe_dev
 * @param atti
 * @param std
 * @return
 */
rgioe_error_t rgioe_get_atti(uint8_t *rgioe_dev, float atti[3], float *std);

rgioe_error_t rgioe_get_pos(uint8_t *rgioe_dev, double pos[3], float *std);

rgioe_error_t rgioe_get_vel(uint8_t *rgioe_dev, float vel[3], float *std);

rgioe_status_t rgioe_get_status(uint8_t *rgioe_dev);

rgioe_error_t rgioe_get_result(uint8_t *rgioe_dev,rgioe_nav_pva_t *pva);

rgioe_error_t rgioe_deinit(uint8_t *rgioe_dev);
#if RGIOE_REALTIME_DEBUG == 1
rgioe_error_t rgioe_set_trace(uint8_t *rgioe_dev,int (*trace)(const char *fmt, ...));
#endif


#ifdef __cplusplus
}
#endif
#endif //INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_H_
