# GNSS/INS/ODO Loosely Couple based on EKF

## 依赖
    1. Eigen
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

