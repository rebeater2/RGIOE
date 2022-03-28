//
// Created by rebeater on 7/8/21.
//

/**
 *this file contains Global Configure for this library,so this file should be includes before any others.
 */
#ifndef RGIOE_DEFINE_H
#define RGIOE_DEFINE_H
#define KD_IN_KALMAN_FILTER 0 /* 是否将里程计比例因子加入卡尔曼滤波状态量*/

#ifndef USE_INCREMENT
#define USE_INCREMENT 1 /*增量数据1 */
#endif

#define ESTIMATE_GNSS_LEVEL_ARM 0    /*将杆臂误差作为状态*/
#define ESTIMATE_ODOMETER_SCALE_FACTOR 0 /*将里程计比例因子作为状态*/

#define BASE_STATE 15

#if ESTIMATE_GNSS_LEVEL_ARM == 1
#define STATE_GNSS_LEVEL_ARM_SIZE 3
#define STATE_GNSS_LEVEL_ARM_START (BASE_STATE)
#else
#define STATE_GNSS_LEVEL_ARM_SIZE 0
#define STATE_GNSS_LEVEL_ARM_START (BASE_STATE)
#endif

#if ESTIMATE_ODOMETER_SCALE_FACTOR == 1
#define STATE_ODOMETER_SCALE_FACTOR_SIZE 1
#define STATE_ODOMETER_SCALE_FACTOR_START ((BASE_STATE)+ STATE_GNSS_LEVEL_ARM_SIZE)
#else
#define STATE_ODOMETER_SCALE_FACTOR_SIZE 0
#endif

/*总状态量*/
#define STATE_CNT  (BASE_STATE+STATE_ODOMETER_SCALE_FACTOR_SIZE+STATE_GNSS_LEVEL_ARM_SIZE)
#define IMU_FRAME 0  /*IMU数据坐标系，1：RFU 0:FRD*/
#define RUN_IN_STM32 0
#define PRINT_MACRO_HELPER(x)   #x
#define PRINT_MACRO(x)   #x "=" PRINT_MACRO_HELPER(x)
#pragma message(PRINT_MACRO(STATE_CNT))
#endif //RGIOE_DEFINE_H