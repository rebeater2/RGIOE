//
// Created by linfe on 2023/7/22.
//
#include <cstdint>

#ifndef RGIOE_RECORDERTYPE_H
#define RGIOE_RECORDERTYPE_H

#include "Recorder.h"

#define RECORDER_MSG_DEBUG 0x16
#define RECORDER_MSG_SENSOR 0x17
#define RECORDER_MSG_RESULT 0x18
#pragma pack(1)
RECORDER_MSG_DEF(RECORDER_MSG_SENSOR, 0x21, imu,
                 {
                     float acce[3];
                     float gyro[3];
                 }
)
#define RECORDER_imu_ITEMS \
{               \
{RECORDER_TYPE_float,"acce_raw_x"},\
{RECORDER_TYPE_float,"acce_raw_y"},\
{RECORDER_TYPE_float,"acce_raw_z"},\
{RECORDER_TYPE_float,"gyro_raw_x"},\
{RECORDER_TYPE_float,"gyro_raw_y"},\
{RECORDER_TYPE_float,"gyro_raw_z"} ,\
}
RECORDER_MSG_DEF(RECORDER_MSG_SENSOR, 0x22, gnss ,
                 {
                     float pos[3];
                     float hdop;
                     float pdop;
                     uint16_t ns;
                     uint8_t mode;
                     uint32_t gnss_cnt;
                 }
)
#define RECORDER_gnss_ITEMS \
{               \
{RECORDER_TYPE_float,"pos_x"},\
{RECORDER_TYPE_float,"pos_y"},\
{RECORDER_TYPE_float,"pos_z"},\
{RECORDER_TYPE_float,"hdop"},\
{RECORDER_TYPE_float,"pdop"},\
{RECORDER_TYPE_uint16_t,"ns"},\
{RECORDER_TYPE_uint8_t,"mode"},\
{RECORDER_TYPE_uint32_t,"gnss_cnt"} ,\
}

RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x21, kalman,
                 {
                     float matP[15];
                     float acce_bias[3];
                     float acce_scale[3];
                     float gyro_bias[3];
                     float gyro_scale[3];
                     float ak;
                     int32_t update_iter;
                     uint32_t feedback_cnt;
                     uint32_t reject_cnt;
                 }
)
#define RECORDER_kalman_ITEMS \
{               \
{RECORDER_TYPE_float,"matP_0"},\
{RECORDER_TYPE_float,"matP_1"},\
{RECORDER_TYPE_float,"matP_2"},\
{RECORDER_TYPE_float,"matP_3"},\
{RECORDER_TYPE_float,"matP_4"},\
{RECORDER_TYPE_float,"matP_5"},\
{RECORDER_TYPE_float,"matP_6"},\
{RECORDER_TYPE_float,"matP_7"},\
{RECORDER_TYPE_float,"matP_8"},\
{RECORDER_TYPE_float,"matP_9"},\
{RECORDER_TYPE_float,"matP_10"},\
{RECORDER_TYPE_float,"matP_11"},\
{RECORDER_TYPE_float,"matP_12"},\
{RECORDER_TYPE_float,"matP_13"},\
{RECORDER_TYPE_float,"matP_14"},\
{RECORDER_TYPE_float,"acce_bias_x"},\
{RECORDER_TYPE_float,"acce_bias_y"},\
{RECORDER_TYPE_float,"acce_bias_z"},\
{RECORDER_TYPE_float,"acce_scale_x"},\
{RECORDER_TYPE_float,"acce_scale_y"},\
{RECORDER_TYPE_float,"acce_scale_z"},\
{RECORDER_TYPE_float,"gyro_bias_x"},\
{RECORDER_TYPE_float,"gyro_bias_y"},\
{RECORDER_TYPE_float,"gyro_bias_z"},\
{RECORDER_TYPE_float,"gyro_scale_x"},\
{RECORDER_TYPE_float,"gyro_scale_y"},\
{RECORDER_TYPE_float,"gyro_scale_z"},\
{RECORDER_TYPE_float,"ak"},         \
{RECORDER_TYPE_int32_t,"update_iter"},\
{RECORDER_TYPE_uint32_t,"feedback_cnt"},\
{RECORDER_TYPE_uint32_t,"reject_cnt"},\
}
RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x22, align,
                 {
                     float atti[3];
                     float vn[3];
                     float pos[3];
                     float v_norm;
                     float a_norm;
                     uint8_t is_staic;
                     uint8_t level_align_finished;
                     uint8_t yaw_align_finished;
                 }
)



#define RECORDER_align_ITEMS \
{                       \
{RECORDER_TYPE_float,"atti_0"},\
{RECORDER_TYPE_float,"atti_1"},\
{RECORDER_TYPE_float,"atti_2"},\
{RECORDER_TYPE_float,"vn_0"},\
{RECORDER_TYPE_float,"vn_1"},\
{RECORDER_TYPE_float,"vn_2"},\
{RECORDER_TYPE_float,"pos_0"},\
{RECORDER_TYPE_float,"pos_1"},\
{RECORDER_TYPE_float,"pos_2"},\
{RECORDER_TYPE_float,"v_norm"},\
{RECORDER_TYPE_float,"a_norm"},\
{RECORDER_TYPE_uint8_t,"is_staic"},\
{RECORDER_TYPE_uint8_t,"level_finished"},\
{RECORDER_TYPE_uint8_t,"yaw_finished"}\
}

RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x23, meas_pos,
                 {
                     float z[3];
                     float r[3];
                     float ak;
                 }
)
#define RECORDER_meas_pos_ITEMS \
{               \
{RECORDER_TYPE_float,"z_0"},\
{RECORDER_TYPE_float,"z_1"},\
{RECORDER_TYPE_float,"z_2"},\
{RECORDER_TYPE_float,"r_0"},\
{RECORDER_TYPE_float,"r_1"},\
{RECORDER_TYPE_float,"r_2"},\
{RECORDER_TYPE_float,"ak"},\
}

RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x24, state,
                 {
                     float xd[21];
                 }
)
#define RECORDER_state_ITEMS \
{               \
{RECORDER_TYPE_float,"Xd_0"},\
{RECORDER_TYPE_float,"Xd_1"},\
{RECORDER_TYPE_float,"Xd_2"},\
{RECORDER_TYPE_float,"Xd_3"},\
{RECORDER_TYPE_float,"Xd_4"},\
{RECORDER_TYPE_float,"Xd_5"},\
{RECORDER_TYPE_float,"Xd_6"},\
{RECORDER_TYPE_float,"Xd_7"},\
{RECORDER_TYPE_float,"Xd_8"},\
{RECORDER_TYPE_float,"Xd_9"},\
{RECORDER_TYPE_float,"Xd_10"},\
{RECORDER_TYPE_float,"Xd_11"},\
{RECORDER_TYPE_float,"Xd_12"},\
{RECORDER_TYPE_float,"Xd_13"},\
{RECORDER_TYPE_float,"Xd_14"},\
{RECORDER_TYPE_float,"Xd_15"},\
{RECORDER_TYPE_float,"Xd_16"},\
{RECORDER_TYPE_float,"Xd_17"},\
{RECORDER_TYPE_float,"Xd_18"},\
{RECORDER_TYPE_float,"Xd_19"},\
{RECORDER_TYPE_float,"Xd_20"},\
}
RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x25, result,
                 {
                     uint32_t status;
                     float pos[3];
                     float vn[3];
                     float atti[3];
                 }
)
#define RECORDER_result_ITEMS \
{               \
{RECORDER_TYPE_uint32_t,"status"},\
{RECORDER_TYPE_float,"pos_x"},\
{RECORDER_TYPE_float,"pos_y"},\
{RECORDER_TYPE_float,"pos_z"},\
{RECORDER_TYPE_float,"vn_x"},\
{RECORDER_TYPE_float,"vn_y"},\
{RECORDER_TYPE_float,"vn_z"},\
{RECORDER_TYPE_float,"atti_x"},\
{RECORDER_TYPE_float,"atti_y"},\
{RECORDER_TYPE_float,"atti_z"},\
}
RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x26, time_check,
                 {
                     float delta_time;
                     float average_delta_time;
                     float max_delta_time;
                     float gnss_delay_time;
                 }
)
#define RECORDER_time_check_ITEMS \
{               \
{RECORDER_TYPE_float,"delta_time"},\
{RECORDER_TYPE_float,"ave_delta_time"},\
{RECORDER_TYPE_float,"max_delta_time"},\
{RECORDER_TYPE_float,"gnss_delay_time"},\
}

RECORDER_MSG_DEF(RECORDER_MSG_DEBUG, 0x27, system,
                 {
                     float gnss_time_delay;
                 }
)
#define RECORDER_system_ITEMS \
{               \
{RECORDER_TYPE_float,"gnss_time_delay"},\
}


RECORDER_MSG_DEF(RECORDER_MSG_DEBUG,0x28, ahrs,
            {
                float roll;
                float pitch;
                float heading;
            }
)
#define RECORDER_ahrs_ITEMS \
{               \
{RECORDER_TYPE_float,"roll"},\
{RECORDER_TYPE_float,"pitch"},\
{RECORDER_TYPE_float,"heading"},\
}

#pragma pack()


#define RECORDER_HEADERCONFIG \
 {                        \
 RECORDER_ADD_DATASET(imu,RECORDER_imu_ITEMS); \
 RECORDER_ADD_DATASET(gnss,RECORDER_gnss_ITEMS); \
 RECORDER_ADD_DATASET(kalman,RECORDER_kalman_ITEMS); \
 RECORDER_ADD_DATASET(align,RECORDER_align_ITEMS); \
 RECORDER_ADD_DATASET(meas_pos,RECORDER_meas_pos_ITEMS); \
 RECORDER_ADD_DATASET(state,RECORDER_state_ITEMS); \
 RECORDER_ADD_DATASET(result,RECORDER_result_ITEMS); \
 RECORDER_ADD_DATASET(time_check,RECORDER_time_check_ITEMS); \
 RECORDER_ADD_DATASET(system,RECORDER_system_ITEMS); \
 RECORDER_ADD_DATASET(ahrs,RECORDER_ahrs_ITEMS); \
}

#endif //RGIOE_RECORDERTYPE_H
