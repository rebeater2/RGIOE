# RGIOE 数据格式说明-CN
## 1. 程序配置文件
程序配置文件可以通过终端调用，YAML 格式为配置文件，系统中两个地方用到了yaml配置，1. 程序配置文件 2 惯导参数

```bash
postDatafusion <file>.yml
```
也可以通过UI程序读取UiDataFusion->file->open或者生成UiDataFusion->file->save

完整的程序配置文件包含以下几个部分：

### 数据时间判断
start-time:开始时间

stop-time:结束时间，-1则代表一直跑到数据结束

### IMU-Config： IMU数据和文件配置
```yaml
 IMU-Config:
  enable: false
  file-path: /media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04cpt.log.imd
  d-rate: 100
  format: 1
  parameter-path: /home/rebeater/CLionProjects/RGIOE/yaml/cpt_parameter.yml
  frame: 0
```
enable: bool无效

file-path: string IMU文件路径

d-rate: int IMU频率, 时间间隔采用固定值，本应该从文件利用IMU的时间戳计算，由于实测数据存在丢帧，因此暂时考虑固定此值

format: int IMU文件格式，0 代表文本文件，后缀用imutxt，1代表二进制文件，后缀为imd，在imu文件格式部分会介绍文件内容定义

parameter-path: string IMU参数文件，

frame: IMU数据坐标系，0表示前右下坐标系，1表示右前上坐标系

### GNSS-Config: GNSS文件和参数配置文件
```yaml
GNSS-Config:
  enable: true
  file-path: /media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482.log.gnsstxt
  format: 1
  std-scale: 0.1
  level-arm:
    - -0.107
    - -0.145
    - -0.388
  double-antenna-enable: false
  antenna-pitch: 0
  antenna-yaw: 0
  columns:
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
    - 0
```
enable: 是否启用GNSS，暂时没有考虑为false的情况

file-path: 文件路径

format：GNSS文件格式，一般是文本文件，需要包含不同的信息：GPS周，GPS秒，纬度、经度、高度、北向定位误差标准差、东向定位误差标准差、地向定位误差标准差、定位模式等信息，通过columns指定，也可以设置特定的文件格式

std-scale: 对GNSS定位误差中标标准差进行缩放，乘以此数构成量测协方差矩阵

level-arm: GNSS天线杆臂

double-antenna-enbale:双天线模式，双天线可以用于初始对准和行进中航向观测，暂时没有合并进主分支

antenna-pitch: 双天线俯仰安装角

antenna-yaw:双天线航向安装角

### Odometer-Config：里程计配置
```yaml
Odometer:
  enable: false
  odometer-std: 0.100000012
  file-path: /media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.veltxt
  NHC-enable: false
  NHC-std:
    - 0
    - 0
  angle-bv:
    - 0
    - 0
    - 0
  wheel-level-arm:
    - 0
    - 0
    - 0
  scale-factor: 0.80000005
  scale-factor-std: 0
```
enable: 是否使用里程计数据

file-path: 里程计文件路径

odometer-std: 里程计观测标准差

NHC-enable: 是否打开非完整性约束观测，对于车载，可以假设侧向和天向速度为0，增加运动约数

NHC-std: 测向速度为0的标准差

angle\_bv: imu坐标系到车体坐标系的安装角

wheel-level-arm: 车轮/里程计 杆臂

scale-factor: 里程计数据比例因子

scale-factor-std: 0 里程计比例因子标准差，在滤波中估计比例因子标准差

### ZUPT-Config 零速检测和更新配置
```yaml
ZUPT-Config:
  ZUPT-enable: false
  ZUPTA-enable: false
  zupt-std: 0
  zupta-std: 0
  zupt-window: 0
  static-wide: 0
  threshold: 0
```
ZUPT-enable: 启用ZUPT

ZUPTA-enable:启用ZUPTA

zupt-std:零速观测标准差

zupt-window/static-wide/threshold：

### Align-Config: 初始对准配置
```yaml
Align-Config:
  mode: 0
  velocity-threshold: 1
  init-PVA:
    week: 0
    gpst: 0
    lat: 0
    lon: 0
    height: 0
    vn:
      - 0
      - 0
      - 0
    atti:
      - 0
      - 0
```
初始对准配置

mode：对准模式

velocity-threshold: 对准速度阈值

int-PVA:当USE\_GIVEN模式，人为设置初始位姿

### Outage-Config: 中断GNSS评估模式
```yaml
Outage-Config:
  enable: false
  start: 122671
  stop: 124000
  step: 120
  outage: 30
```
enable: 是否启用

start: 开始中断时间

stop:停止中断模式时间

step: 每次评估间隔时间

outage:每次中断时间

### Output-Config: 输出模式配置
```yaml
Output-Config:
  file-path: /media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482_cpt.nav
  file-format: 1
  project-enable: true
  position-project:
    - 0.21
    - -0.05
    - -0.418
  attitude-project:
    - 0
    - -0.448
    - 3.32
```
file-path: 输出文件路径

file-format: 输出文件路径

position-project: 输出位置投影,单位m, 有精力可以改成四位置法或者六位置法

attitude-project: 输出姿态投影

### Pressure-Config气压计参数配置
```yaml
  enable: false
  file-path: /media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.bmptxt
  height-std: 0.2
```
enable:是否启用

file-path: 文件路径

height-std: 高程观测标准差，单位 m
# 文件格式说明
### IMU文件说明
IMU-Config中的format为0时，代表输入文件为imutxt格式，imutxt是以ASCII文本存储的IMU数据文件，内容为数字矩阵，由7列构成，空格隔开，CRLF换行，每列的含义如下:

|列序|含义|单位|
| ----- | ----- | ----- |
|1|GPS周内秒|秒|
|2\~4|陀螺仪数据x,y,z轴|rad/s |
|5\~7|加速度计数据x,y,z轴|g(当地重力)|

注意 没有保存GPS周信息，因此不能直接转换为UTC时间。

format为1时，代表imd文件，规定imd文件为double类型数据行优先二进制排列（小端模式），内容和imutxt文件一致，一般保存为增量模式，即数值的含义为当前时刻的变化量，陀螺数据单位为rad，加速度计单位为m/s，（已经乘以dt）

## GNSS文件说明
GNSS文件为文本文件，必须包含的内容应该包括GPS周、GPS秒、纬度、经度、高度（大地高）、北向定位标准差、东向定位标准差、地向定位标准差，定位模式，可以从接收机的GPGGA和GPGXT语句中得到。

### 里程计文件说明
文本文件，两列，第一列为GPS周内秒、第二列为当前时刻速度，单位为m/s。

### 气压计文件说明
文本文件 两列，第一列为GPS周内秒，第二列为气压数据，单位Pa

实际使用的时候可能由于设备原因格式不完全一致，可以参考FileIO.cpp和FileIO.h中相关reader类，对其继承

