//
// Created by linfe on 2023/7/30.
//

#ifndef STATICGRAPH_RECORDERHEADERDEF_H
#define STATICGRAPH_RECORDERHEADERDEF_H
#define SENSOR_DATA_ITEMS \
{               \
{RECORDER_TYPE_float,"acce_raw_x"},\
{RECORDER_TYPE_float,"acce_raw_y"},\
{RECORDER_TYPE_float,"acce_raw_z"},\
{RECORDER_TYPE_float,"acce_fix_x"},\
{RECORDER_TYPE_float,"acce_fix_y"},\
{RECORDER_TYPE_float,"acce_fix_z"},\
{RECORDER_TYPE_float,"gyro_raw_x"},\
{RECORDER_TYPE_float,"gyro_raw_y"},\
{RECORDER_TYPE_float,"gyro_raw_z"} ,\
{RECORDER_TYPE_float,"gyro_fix_x"},\
{RECORDER_TYPE_float,"gyro_fix_y"},\
{RECORDER_TYPE_float,"gyro_fix_z"}\
}
#define KALMAN_DATA_ITEMS \
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
{RECORDER_TYPE_float,"matP_15"},\
{RECORDER_TYPE_float,"matP_16"},\
{RECORDER_TYPE_float,"matP_17"},\
{RECORDER_TYPE_float,"matP_18"},\
{RECORDER_TYPE_float,"matP_19"},\
{RECORDER_TYPE_float,"matP_20"},\
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
}

#define STATE_ITEMS \
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

#define meas_pos_ITEM_DEF \
{               \
{RECORDER_TYPE_float,"z_0"},\
{RECORDER_TYPE_float,"z_1"},\
{RECORDER_TYPE_float,"z_2"},\
{RECORDER_TYPE_float,"r_0"},\
{RECORDER_TYPE_float,"r_1"},\
{RECORDER_TYPE_float,"r_2"},\
{RECORDER_TYPE_float,"r_3"},\
}


#define HEADERCONFIG(target) \
 do {                        \
 RECORDER_ADD_DATASET(target,recorder_msg_imu_id,"sensor_data",SENSOR_DATA_ITEMS); \
 RECORDER_ADD_DATASET(target,recorder_msg_kalman_id,"ekf",KALMAN_DATA_ITEMS);  \
 RECORDER_ADD_DATASET(target,recorder_msg_state_id,"state",STATE_ITEMS);  \
 RECORDER_ADD_DATASET(target,recorder_msg_meas_pos_id,"meas_pos",meas_pos_ITEM_DEF);  \
    }while(0)                         \
    \



#endif //STATICGRAPH_RECORDERHEADERDEF_H
