//
// Created by rebeater on 7/8/21.
//

#define KD_IN_KALMAN_FILTER 1 /* 是否将里程计比例因子加入卡尔曼滤波状态量*/

#if KD_IN_KALMAN_FILTER == 1
#define STATE_CNT  16
#else
#define STATE_CNT  15
#endif
#define IMU_FRAME 1/*IMU数据坐标系，1：RFU 0:FRD*/