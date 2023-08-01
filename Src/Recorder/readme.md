# Recorder 项目 readme
# a light Variable recorder system
main functions:

1. record variables into rcd file according to timestamp
2. load and parse rcd file into `std::map<uint32_t,std::vector<float>>` to be plot in matlab, python, qwt, matplotlib, or,save into txt

rcd file definition:
header:

|offset|size(byte)|type|introduction|
| ----- | ----- | ----- | ----- |
|0|4|0XBB55BB66L|start of header|
|4|4|uint32\_t|M: number of datasets|
|8|4|uint32\_t|ID of dataset\[0\]|
|12|4|uint32\_t|N0: number of subdatasets for dataset\[0\]|
|16|16|char\[16\]|name of datasets\[0\]|
|32|4|enum `RecorderBaseType`|type id for dataset\[0\].item\[0\]|
|36|16|char\[16\]|name of datasets\[0\].item\[0\]|
|52|4|enum `RecorderBaseType`|type id for dataset\[0\].item\[1\]|
|36|16|char\[16\]|name of datasets\[0\].item\[1\]|
|......|......|......|......|
|32+20\*(N0-1)|4|enum `RecorderBaseType`|type id for dataset\[0\].item\[N-1\]|
|36+20\*(N0-1)|16|char\[16\]|name of datasets\[0\].item\[N-1\]|
|52+20\*(N0-1)|4|uint32\_t|ID of dataset\[1\]|
|56+20\*(N0-1)|4|uint32\_t|N1: number of subdatasets for dataset\[1\]|
|60+20\*(N0-1)|16|char\[16\]|name of datasets\[1\]|
|......|......|......|......|
|72+20*(N0-1) + 20(N0-1)...*|4| |ID of dataset\[M-1\]|
|88+20*(N0-1) + 20(N0-1)...*|4| |N: number of subdatasets for dataset\[M-1\]|
|72+20*(N0-1) + 20(N0-1)...*|16| |name of datasets\[M-1\]|
| |uint32\_t|0XBB77BB88L|end of header|

body:

|offset|memeber|size(byte)|type|introduction|
| ----- | ----- | ----- | ----- | ----- |
| |header|4|uint32\_t|0XAA55AA55L|
| |recorder\_id|4|uint32\_t|dataset id|
| |timestamp|8|uint64\_t |timestamp|
| |length|4|uint32\_t|length of body elements|
| |data| | | |
| |check\_sum|4|uint32\_t|crc32 checksum|
| |end\_mark|4|uint32\_t|0XAA66AA66L|

this id defined by:

```cpp
#define RECORDER_MSG_DEF(set, id, name, fmt) \
typedef struct  {                            \
uint32_t header;                              \
uint32_t msg_id ;                             \
uint64_t timestamp;             \
uint32_t length;                \
struct fmt  data;                 \
uint32_t check_sum;             \
uint32_t end_mark;              \
} recorder_msg_##name##_t ;     \
enum{recorder_msg_##name##_id = MSG_ID(set,id)};
```
### example
step1: define a recorder struct(recommended in `RecorderType.h`)
```c++
RECORDER_MSG_DEF(RECORDER_SET_SENSOR, 0x21, imu,
                  {
                      float gyro[3];
                      float acce[3];
                      float gyro_fix[3];
                      float acce_fix[3];
                  }
)
```
where `RECORDER_SET_SENSOR`, `0x21` are used to generate message ID.

step2: define and register a header(recommanded in `RecorderHeaderDef.h`) for every item
```C++
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
```
and add configure to `HEADERCONFIG`
```C++
#define HEADERCONFIG(target) \
 do {                        \
 RECORDER_ADD_DATASET(target,recorder_msg_imu_id,"sensor_data",SENSOR_DATA_ITEMS); \
 RECORDER_ADD_DATASET(target,recorder_msg_kalman_id,"ekf",KALMAN_DATA_ITEMS);  \
 RECORDER_ADD_DATASET(target,recorder_msg_state_id,"state",STATE_ITEMS);  \
 RECORDER_ADD_DATASET(target,recorder_msg_meas_pos_id,"meas_pos",meas_pos_ITEM_DEF);  \
    }while(0)
```

step3: Create a Recorder object and call `recorder.WriteHeader()` in start
```C++
Recorder::GetInstance().Initialize(argv[0]);
```
Recorder a struct while program is running:
```C++
    recorder_msg_imu_t imu_data = CREATE_RECORDER_MSG(imu);
    imu_data.timestamp = *(uint64_t *) &nav.gpst;
    for (int i = 0; i < 3; ++i) {
        imu_data.data.gyro[i] = imu.gyro[i];
        imu_data.data.acce[i] = imu.acce[i];
    }
    CHECKSUM_RECORDER_CRC32(&imu_data);/*Don't forget this*/
    Recorder::GetInstance().Record<recorder_msg_imu_t>(&imu_data);
```
