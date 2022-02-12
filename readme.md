# RGIOE readme.md
# RGIOE:A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter
A Real-time GNSS/INS/Odometer Integrated Navigation Algorithm based on Extend Kalman Filter. 

# Description
This is an implemention of GNSS/INS/Odometer multi-sensor fusion algorithm base on extend Kalman filter. A high precise inertial mechanization is also  implemented. We also take account of NHC and ZUPT. A C-tyle interface is provided for embedded applicaitons.  We designed a console applicaiton  and a UI version for a post-processing.  A real-time simulation based on ROS will be implemented soon. Brief ad follows:

1. Loosely coupled algorithm
2. High precise mechanization
3. Real-time and post-processing supported
4. EKF based

# Complile
The algorithm library (in Inc and Src  directory) is dependent and only relies on Eigen. To compile the library, you should install eigen on your computer by `apt install libeigen` ,or download its code and tell the compliter where it is by `include_directory` in CMakelist.txt

To complile the exeutable program, you need to install the following library and all of them can be obtained by package manager :

1. [fmt](https://github.com/fmtlib/fmt.git)
2. [yaml-cpp](https://github.com/jbeder/yaml-cpp.git)
3. [glog](https://github.com/google/glog.git)
4. [Qt](https://www.qt.io/)

# Usage
Both the execuable console program and UI program is able to load the configure file in format of YAML. Some demo configure file are provided in directory `yaml` .  In terminal, you can use the following command:

```Plain Text
dataFusion <configure.yml>
```
Or in UiDataFusion, you can use file->load to choose an configure. some of this feature is under development.

# C style API
To use this library in C environment such as STM32 and DSP, C style functions are provided.

```cpp
int navInitialize();
int navOutput(NavOutput *nav_output);
double navAlignGnss(const GnssData *gnss);
int navAlignLevel(const ImuData *imu);
void navSetGNSS(const GnssData *gnss);
void navSetVel(const Velocity *vel);
void navUpdate(const ImuData *imu);
```
# Performance
Test IMU : POS830 FOG

ARW: 0.003, deg/s/sqrt(hr)   

VRW: 0.03, m/s/sqrt(hr)   

gyroscope bias stability: 0.027, deg/hr   

accelorator bias stability: 15, mGal     

| |x error /m|y error/m|z error/m|
| ----- | ----- | ----- | ----- |
|position error(m) 1-$\sigma$|0.0066|0.0116|0.0098|
|position error(m) 2-$\sigma$|0.0194|0.0220|0.0266|
|position error(m) rms|0.0085|0.0112|0.0133|
|velocity error(m/s) 1-$\sigma$|0.0025|0.0018|0.0018|
|velocity error(m/s) 2-$\sigma$|0.0058|0.0045|0.0056|
|velocity error(m/s) rms|0.0027|0.0021|0.0028|
|attitude error(deg) 1-$\sigma$|0.0008|0.0009|0.0238|
|attitude error(deg) 2-$\sigma$|0.0016|0.0018|0.0270|
|attitude error(deg) rms|0.0008|0.0009|0.0180|
|2D error 1-$\sigma$|0.0149| | |
|2D error 2-$\sigma$|0.0253| | |
|2D error RMS|0.0141| | |
|3D error 1-$\sigma$|0.0187| | |
|3D error 2-$\sigma$|0.0326| | |
|3D error RMS $\sigma$|0.0194| | |



# 
