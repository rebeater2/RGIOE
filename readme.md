# GNSS/INS/ODO Loosely Couple based on EKF

## 算法核心依赖
    1. Eigen
## 测试应用依赖
    2. glog
    3. gfalgs
    4. yaml-cpp
## Usage
```
dataFusion configure.yml
```

## 格式说明
#### GNSS数据格式
function:
```c++
ifstream &operator>>(ifstream &is, GnssData &gnss) {
  is >> gnss.week >> gnss.gpst;
  is >> gnss.lat >> gnss.lon >> gnss.height;
  is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
  is >> gnss.hdop >> gnss.ns >> gnss.mode;
  return is;
}
```
其他格式可以通过修改此函数重新定义格式

单天线数据格式如下

| name       | unit   | colulms | commnets   |
| ---------- | ------ | ------- | ---------- |
| week       | int    | [0]     |            |
| gps second | second | [1]     |            |
| latitude   | deg    | 2       |            |
| lontitude  | deg    | 3       |            |
| height     | m      | 4       |            |
| pos std N  | m      | 5       | from GNGST |
| pos std E  | m      | 6       |            |
| pos std D  | m      | 7       |            |
| hdop       | m      | 8       |            |
| ns         |        | 9       |            |
| mode       |        | 10      |            |

IMU数据格式

文本数据可以通过以下函数读取

```c++
ifstream &operator>>(ifstream &is, ImuData &imu) {

#if IMU_FRAME == 0 /*前左上*/
  is >> imu.gpst;
  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
#else /*右前下坐标系*/
  is >> imu.gpst;
  is >> imu.gyro[1] >> imu.gyro[0] >> imu.gyro[2];
  is >> imu.acce[1] >> imu.acce[0] >> imu.acce[2];
  imu.gyro[2] *= (-1.0);
  imu.acce[2] *= (-1.0);
#endif
  return is;
}
```



| name               | unit   | columns | comments         |
| ------------------ | ------ | ------- | ---------------- |
| gps second         | second | 0       |                  |
| gyrosope x axis    | rad/s  | 1       | FRD or RFU坐标系 |
| gyrosope y axis    | rad/s  | 2       |                  |
| gyrosope z axis    | rad/s  | 3       |                  |
| accelerator x axis | g      | 4       |                  |
| accelerator y axis | g      | 5       |                  |
| accelerator z axis | g      | 6       |                  |

odometer数据格式

读取函数

```c++
istream &operator>>(istream &is, AuxiliaryData &aux) {
  is >> aux.gpst >> aux.velocity >> aux.angular_vel;
  return is;
}
```

|                  | unit   | columns | comments |
| ---------------- | ------ | ------- | -------- |
| gps second       | second | 0       |          |
| forward velocity | m/s    | 1       |          |
| z-axis velocity  | deg/s  | 2       |          |

输出数据格式可以在`FileIO`中配置

```c++
ostream &operator<<(ostream &os, const NavOutput &output) {
  os << output.week << SEPERATE <<fixed<< setprecision(3) << output.gpst << SEPERATE;
  os << fixed << setprecision(12) << output.pos[0] / _deg << SEPERATE << output.pos[1] / _deg << SEPERATE;
  os << fixed << setprecision(3) << output.pos[2] << SEPERATE;
  os << fixed << setprecision(3) << output.vn[0] << SEPERATE << output.vn[1] << SEPERATE << output.vn[2] << SEPERATE;
  os << fixed << setprecision(3) << output.atti[0] / _deg << SEPERATE << output.atti[1] / _deg << SEPERATE
	 << output.atti[2] / _deg << SEPERATE;
  os << fixed << setprecision(2) << output.gb[0] / _deg * _hour << SEPERATE << output.gb[1] / _deg * _hour << SEPERATE
	 << output.gb[2] / _deg * _hour << SEPERATE;
  os << fixed << setprecision(2) << output.ab[0] / _deg * _hour << SEPERATE << output.ab[1] / _deg * _hour << SEPERATE
	 << output.ab[2] / _deg * _hour << SEPERATE;
  os << output.info.gnss_mode << SEPERATE << output.info.sensors << SEPERATE;
#if KD_IN_KALMAN_FILTER == 1
  os << fixed << setprecision(3) << output.kd << SEPERATE;
#endif
  return os;
}
```



| name        | unit   | index | commnets                 |
| ----------- | ------ | ----- | ------------------------ |
| week        |        | 0     |                          |
| GPS second  | second | 1     |                          |
| latitude    | deg    | 2     |                          |
| longtitude  | deg    | 3     |                          |
| height      | m      | 4     | NOT altitude             |
| vn -x       | m/s    | 5     | velocity in n-frame      |
| vn-y        | m/s    | 6     |                          |
| vn-z        | m/s    | 7     |                          |
| atti x      | deg    | 8     | roll                     |
| atti y      | deg    | 9     | pitch                    |
| atti z      | deg    | 10    | yaw                      |
| acce bias x | mGal   | 11    | accelerator bias         |
| acce bias y | mGal   | 12    |                          |
| acce bias z | mGal   | 13    |                          |
| gyro bias x | deg/h  | 14    |                          |
| gyro bias y | deg/h  | 15    |                          |
| gyro bias z | deg/h  | 16    |                          |
| GNSS mode   |        | 17    | same as GNGGA            |
| sensor type |        | 18    | 传感器类型               |
| kd          |        | 19    | 里程计比例因子，当有效时 |

性能 激光惯导+天宝接收机RTK模式

测试陀螺性能

IMU参数:
角度随机游走: 0.003, deg/s/sqrt(hr)   
速度随机游走: 0.03, m/s/sqrt(hr)   
陀螺零偏标准差: 0.027, deg/hr   
加表零偏标准差: 15, mGal   
陀螺零偏相关时间: 4, hr   
加表零偏相关时间: 4, hr   
陀螺比例因子标准差: 300, ppm   
加表比例因子标准差: 300, ppm   
陀螺比例因子相关时间: 4, hr   
加表比例因子相关时间: 4, hr   



|                                | x error /m | y error/m | z error/m |
|--------------------------------|------------|-----------|-----------|
|  position error(m) 1-$\sigma$  |   0.0193   |   0.0181  |   0.0144  |
|  position error(m) 2-$\sigma$  |   0.0694   |   0.0694  |   0.0347  |
|     position error(m) rms      |   0.0291   |   0.0271  |   0.0234  |
| velocity error(m/s) 1-$\sigma$ |   0.0144   |   0.0030  |   0.0030  |
| velocity error(m/s) 2-$\sigma$ |   0.0347   |   0.0079  |   0.0071  |
|    velocity error(m/s) rms     |   0.0234   |   0.0036  |   0.0033  |
| attitude error(deg) 1-$\sigma$ |   0.0020   |   0.0035  |   0.0035  |
| attitude error(deg) 2-$\sigma$ |   0.0040   |   0.0053  |   0.0054  |
|    attitude error(deg) rms     |   0.0029   |   0.0031  |   0.0031  |
|      2D error 1-$\sigma$       |   0.0405   |           |           |
|      2D error 2-$\sigma$       |   0.0764   |           |           |
|         2D error RMS$          |   0.0398   |           |           |
|      3D error 1-$\sigma$       |   0.0476   |           |           |
|      3D error 2-$\sigma$       |   0.0822   |           |           |
|     3D error RMS $\sigma$      |   0.0462   |           |           |
|--------------------------------|------------|-----------|-----------|