/**
* @file RgioeConfig.h in RGIOE
* @author linfe
* @comment 算法配置文件，配置卡尔曼滤波维数
* Create on 2022/5/11 11:44
* @version 1.0
**/

#ifndef RGIOE_INC_RGIOECONFIG_H_
#define RGIOE_INC_RGIOECONFIG_H_

/*增量数据1 */
#define USE_INCREMENT 1

/*将加速度计比例因子作为状态向量*/
#define ESTIMATE_ACCE_SCALE_FACTOR 0

/*将陀螺仪比例因子作为状态向量*/
#define ESTIMATE_GYRO_SCALE_FACTOR 0

/*将杆臂误差作为状态*/
#define ESTIMATE_GNSS_LEVEL_ARM 0

/*将里程计比例因子作为状态*/
#define ESTIMATE_ODOMETER_SCALE_FACTOR 0

#define RGIOE_CONFIG_PRECISE  FP64
#endif //RGIOE_INC_RGIOECONFIG_H_
