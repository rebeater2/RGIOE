/**
* @file RgioeConfig.h in RGIOE
* @author linfe
* @comment 算法配置文件，配置卡尔曼滤波维数, 针对不同的应用，可以复制到不同的文件夹供编译时查找到即可
* Create on 2022/5/11 11:44
* @version 1.0
**/

#ifndef RGIOE_INC_RGIOECONFIG_H_
#define RGIOE_INC_RGIOECONFIG_H_

/*增量数据1 */

/*将加速度计比例因子作为状态向量*/
#define ESTIMATE_ACCE_SCALE_FACTOR 0

/*将陀螺仪比例因子作为状态向量*/
#define ESTIMATE_GYRO_SCALE_FACTOR 0

/*将杆臂误差作为状态*/
#define RGIOE_ESTIMATE_GNSS_LEVEL_ARM 0

/*将里程计比例因子作为状态*/
#define RGIOE_ESTIMATE_ODOMETER_SCALE_FACTOR 0

#ifndef RGIOE_CONFIG_PRECISE
#define RGIOE_CONFIG_PRECISE  FP32
#endif

#ifndef RGIOE_ENABLE_RTS
#define RGIOE_ENABLE_RTS  1
#endif

#endif //RGIOE_INC_RGIOECONFIG_H_
