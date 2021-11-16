//
// Created by rebeater on 7/8/21.
//

/**
 *this file contains Global Configure for this library,so this file should be includes before any others.
 */
#define KD_IN_KALMAN_FILTER 0 /* 是否将里程计比例因子加入卡尔曼滤波状态量*/

#ifndef USE_INCREMENT
#define USE_INCREMENT 1 /*增量数据*/
#endif
#if KD_IN_KALMAN_FILTER == 1
#define STATE_CNT  16
#else
#define STATE_CNT  15
#endif
#define IMU_FRAME 0  /*IMU数据坐标系，1：RFU 0:FRD*/
#define USE_OUTAGE 0 /*GNSS中断评估模式*/