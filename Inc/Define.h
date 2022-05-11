//
// Created by rebeater on 7/8/21.
//

/**
 *this file contains Global Configure for this library,so this file should be includes before any others.
 */
#ifndef RGIOE_DEFINE_H
#define RGIOE_DEFINE_H

#include "RgioeConfig.h"
#define BASE_STATE 15
/*系统状态量组成为 []表示可选：
 * 位置(3) 速度(3) 姿态(3) 加速度零偏(3) 陀螺零偏(3) [陀螺比例因子(3)] [加速度计比例因子] [杆臂(3)] [里程计比例因子(1)]
 * 前15维度必选 后10维可选，最高25维度状态空间
 * 以下代码定义 可选状态空间在状态向量中的起点和所占维度，用于在滤波时索引指定维度
 * */
#if ESTIMATE_GYRO_SCALE_FACTOR == 1
#define STATE_GYRO_SCALE_FACTOR_SIZE 3
#define STATE_GYRO_SCALE_FACTOR_START (BASE_STATE)
#else
#define STATE_GYRO_SCALE_FACTOR_SIZE 0
#endif

#if ESTIMATE_ACCE_SCALE_FACTOR == 1
#define STATE_ACCE_SCALE_FACTOR_SIZE 3
#define STATE_ACCE_SCALE_FACTOR_START (STATE_GYRO_SCALE_FACTOR_START + STATE_GYRO_SCALE_FACTOR_SIZE )
#else
#define STATE_ACCE_SCALE_FACTOR_SIZE 0
#endif

#if ESTIMATE_GNSS_LEVEL_ARM == 1
#define STATE_GNSS_LEVEL_ARM_SIZE 3
#define STATE_GNSS_LEVEL_ARM_START (STATE_ACCE_SCALE_FACTOR_START + STATE_ACCE_SCALE_FACTOR_SIZE)
#else
#define STATE_GNSS_LEVEL_ARM_SIZE 0
#endif

#if ESTIMATE_ODOMETER_SCALE_FACTOR == 1
#define STATE_ODOMETER_SCALE_FACTOR_SIZE 1
#define STATE_ODOMETER_SCALE_FACTOR_START ((STATE_GNSS_LEVEL_ARM_START)+ STATE_GNSS_LEVEL_ARM_SIZE)
#else
#define STATE_ODOMETER_SCALE_FACTOR_SIZE 0
#endif

/*总状态量*/
#define STATE_CNT  (BASE_STATE+STATE_GYRO_SCALE_FACTOR_SIZE + STATE_ACCE_SCALE_FACTOR_SIZE+STATE_ODOMETER_SCALE_FACTOR_SIZE+STATE_GNSS_LEVEL_ARM_SIZE)

#define PRINT_MACRO_HELPER(x)   #x
#define PRINT_MACRO(x)   #x "=" PRINT_MACRO_HELPER(x)
//#pragma message(PRINT_MACRO(STATE_CNT))
#endif //RGIOE_DEFINE_H