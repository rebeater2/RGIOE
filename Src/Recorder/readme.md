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


