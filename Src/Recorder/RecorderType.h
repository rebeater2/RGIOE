/******************************************************************************
 * RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter
 * Copyright (C) 2024                                                         *
 * Author : rebeater                                                          *
 * Contact : rebeater@qq.com                                                  *
 ******************************************************************************/
/**
 * do NOT add #pragma once to this file
 * example:
RECORDER_TYPE_DEF_START(imu, 0x16, 0x21)
RECORDER_ITEM_DEF(float, acce_x)
RECORDER_ITEM_DEF(float, acce_y)
RECORDER_ITEM_DEF(float, acce_z)
RECORDER_ITEM_DEF(float, gyro_x)
RECORDER_ITEM_DEF(float, gyro_y)
RECORDER_ITEM_DEF(float, gyro_z)
RECORDER_TYPE_DEF_END(imu)
 */
#define RECORDER_MSG_DEBUG 0x16
#define RECORDER_MSG_SENSOR 0x17
#define RECORDER_MSG_RESULT 0x18

RECORDER_TYPE_DEF_START(imu, RECORDER_MSG_SENSOR, 0x21)
RECORDER_ITEM_DEF(float, acce_x)
RECORDER_ITEM_DEF(float, acce_y)
RECORDER_ITEM_DEF(float, acce_z)
RECORDER_ITEM_DEF(float, gyro_x)
RECORDER_ITEM_DEF(float, gyro_y)
RECORDER_ITEM_DEF(float, gyro_z)
RECORDER_TYPE_DEF_END(imu)

RECORDER_TYPE_DEF_START(gnss, RECORDER_MSG_SENSOR, 0x22)
RECORDER_ITEM_DEF(float, pos_x)
RECORDER_ITEM_DEF(float, pos_y)
RECORDER_ITEM_DEF(float, pos_z)
RECORDER_ITEM_DEF(float, hdop)
RECORDER_ITEM_DEF(float, pdop)
RECORDER_ITEM_DEF(uint16_t, ns)
RECORDER_ITEM_DEF(uint8_t, mode)
RECORDER_ITEM_DEF(uint32_t, gnss_cnt)
RECORDER_TYPE_DEF_END(gnss)

RECORDER_TYPE_DEF_START(kalman, RECORDER_MSG_DEBUG, 0x23)
RECORDER_ITEM_DEF(float, acce_bias_x)
RECORDER_ITEM_DEF(float, acce_bias_y)
RECORDER_ITEM_DEF(float, acce_bias_z)
RECORDER_ITEM_DEF(float, gyro_bias_x)
RECORDER_ITEM_DEF(float, gyro_bias_y)
RECORDER_ITEM_DEF(float, gyro_bias_z)
RECORDER_TYPE_DEF_END(kalman)

RECORDER_TYPE_DEF_START(meas_pos, RECORDER_MSG_DEBUG, 0x24)
RECORDER_ITEM_DEF(float, z_x)
RECORDER_ITEM_DEF(float, z_y)
RECORDER_ITEM_DEF(float, z_z)
RECORDER_ITEM_DEF(float, r_x)
RECORDER_ITEM_DEF(float, r_y)
RECORDER_ITEM_DEF(float, r_z)
RECORDER_TYPE_DEF_END(meas_pos)


RECORDER_TYPE_DEF_START(result, RECORDER_MSG_DEBUG, 0x25)
RECORDER_ITEM_DEF(uint8_t, status)
RECORDER_ITEM_DEF(float, pos_x)
RECORDER_ITEM_DEF(float, pos_y)
RECORDER_ITEM_DEF(float, pos_z)
RECORDER_ITEM_DEF(float, vn_x)
RECORDER_ITEM_DEF(float, vn_y)
RECORDER_ITEM_DEF(float, vn_z)
RECORDER_ITEM_DEF(float, atti_x)
RECORDER_ITEM_DEF(float, atti_y)
RECORDER_ITEM_DEF(float, atti_z)
RECORDER_TYPE_DEF_END(result)

RECORDER_TYPE_DEF_START(time_check, RECORDER_MSG_DEBUG, 0x26)
RECORDER_ITEM_DEF(float, delta_time)
RECORDER_ITEM_DEF(float, average_time)
RECORDER_ITEM_DEF(float, max_delta_time)
RECORDER_ITEM_DEF(float, gnss_delay_time)
RECORDER_TYPE_DEF_END(time_check)

RECORDER_TYPE_DEF_START(ahrs, RECORDER_MSG_DEBUG, 0x27)
RECORDER_ITEM_DEF(float, roll)
RECORDER_ITEM_DEF(float, pitch)
RECORDER_ITEM_DEF(float, heading)
RECORDER_TYPE_DEF_END(ahrs)

RECORDER_TYPE_DEF_START(align, RECORDER_MSG_DEBUG, 0x28)
RECORDER_ITEM_DEF(float, atti_x)
RECORDER_ITEM_DEF(float, atti_y)
RECORDER_ITEM_DEF(float, atti_z)
RECORDER_ITEM_DEF(float, vn_x)
RECORDER_ITEM_DEF(float, vn_y)
RECORDER_ITEM_DEF(float, vn_z)
RECORDER_ITEM_DEF(float, pos_x)
RECORDER_ITEM_DEF(float, pos_y)
RECORDER_ITEM_DEF(float, pos_z)
RECORDER_ITEM_DEF(float, v_norm)
RECORDER_ITEM_DEF(float, a_norm)
RECORDER_ITEM_DEF(uint8_t, is_static)
RECORDER_ITEM_DEF(uint8_t, yaw_align)
RECORDER_ITEM_DEF(uint8_t, level_align)
RECORDER_TYPE_DEF_END(align)
