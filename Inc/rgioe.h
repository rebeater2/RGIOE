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
#include "rgioe_data.h"


rgioe_error_t rgioe_init(const rgioe_data_t *opt);

rgioe_error_t rgioe_timeupdate(rgioe_imu_inc_t * imu_inc);

rgioe_error_t rgioe_gnssupdate(double timestamp,rgioe_gnss_t *gnss);




#ifdef __cplusplus
};
#endif
#endif //INSCUBEBSP_CORE_SRC_APP_ALGORITHM_RGIOE_INC_RGIOE_H_
