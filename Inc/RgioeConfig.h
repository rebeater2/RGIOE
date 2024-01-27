/**
* @file RgioeConfig.h in RGIOE
* @author linfe
* @comment 算法配置文件，配置卡尔曼滤波维数，积分精度/速度等
* Create on 2022/5/11 11:44
* @version 1.0
**/

#ifndef RGIOE_INC_RGIOECONFIG_H_
#define RGIOE_INC_RGIOECONFIG_H_


/*将加速度计比例因子作为状态向量*/
#ifndef RGIOE_ESTIMATE_ACCE_SCALE_FACTOR
#define RGIOE_ESTIMATE_ACCE_SCALE_FACTOR 0
#endif
/*将陀螺仪比例因子作为状态向量*/
#ifndef RGIOE_ESTIMATE_GYRO_SCALE_FACTOR
#define RGIOE_ESTIMATE_GYRO_SCALE_FACTOR 0
#endif

/*将杆臂误差作为状态*/
#ifndef RGIOE_ESTIMATE_GNSS_LEVEL_ARM
#define RGIOE_ESTIMATE_GNSS_LEVEL_ARM 0
#endif

/*将里程计比例因子作为状态*/
#ifndef RGIOE_ESTIMATE_ODOMETER_SCALE_FACTOR
#define RGIOE_ESTIMATE_ODOMETER_SCALE_FACTOR 0
#endif

/*使用浮点数类型 FP64: double, FP32: float32 */
#ifndef RGIOE_CONFIG_PRECISE
#define RGIOE_CONFIG_PRECISE  FP64
#endif

/* 带RTS， 涉及到STL和大内存容器引入 */
#ifndef RGIOE_ENABLE_RTS
#define RGIOE_ENABLE_RTS  0
#endif

/* 带双天线观测 */
#ifndef RGIOE_ENABLE_DOUBLE_ANTENNA
#define RGIOE_ENABLE_DOUBLE_ANTENNA  0
#endif

/* 使用recorder 记录调试信息 */
#ifndef ENABLE_FUSION_RECORDER
#define ENABLE_FUSION_RECORDER 0
#endif

/* 使用快速积分算法 */
#ifndef RGIOE_FAST_INS
#define RGIOE_FAST_INS 0
#endif

#endif //RGIOE_INC_RGIOECONFIG_H_
