/**
* @file f_io.h in LooselyCouple2020_cpp
* @author rebeater
* @comment file operation
* Create on 2021/1/16 下午3:56
* @version 1.0
**/

//
// Created by rebeater on 2021/1/16.
//

#ifndef LOOSELYCOUPLE2020_CPP_FILEIO_H
#define LOOSELYCOUPLE2020_CPP_FILEIO_H

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <fstream>
#include <condition_variable>

#include "NavStruct.h"

using namespace std;
#define SEPERATE (' ')


ostream &operator<<(ostream &os,const ImuData &imu);

ostream &operator<<(ostream &os,const NavOutput &output);

ifstream &operator>>(ifstream &is, ImuData &imu);
int ReadImu(istream &is,ImuData &imu,IMUFileFormat fmt);


ifstream &operator>>(ifstream &is, GnssData &gnss);

ostream &operator<<(ostream &os,const ImuPara &imuPara);

ostream &operator<<(ostream &os, const AuxiliaryData &aux);
ostream &operator<<(ostream &os,const GnssData &gnss);
 istream &operator>>( istream &is,  AuxiliaryData &aux);

class NavWriter {
  /*多线程读写*/
  queue<std::shared_ptr<NavOutput>> nav_msgs;

  std::thread th_write;
  std::mutex mtx_nav;
  string file_path;
 private:
  atomic_flag flag_running = ATOMIC_FLAG_INIT;
  void th_write_nav();
  void start();

 public:
  explicit NavWriter(string filepath);

  ~NavWriter();

  void stop();

  void update(const NavOutput &out);
};

template<typename T>
class ReaderBase {
 public:
  explicit ReaderBase();
  virtual bool ReadNext(T &data);
  /**
   * 虚函数，返回data所在时间
   * @param data
   * @return 时间（gps time)
   */
  virtual double GetTime(const T &data) const;
  /**
   * 读取下一帧数据，直到数据的GetTime函数达到gpst,将最后一帧数据保存在data里面
   * @param gpst 目标时间
   * @param data 如果为nullptr则不保存数据
   * @return 是否成功，失败原因：1，文件打开失败，2 文件读取完毕
   */
  bool ReadUntil(double gpst, T *data = nullptr);

  bool IsOk() const;
 protected:
  ifstream ifs;
  bool ok_;
};

template<typename T>
bool ReaderBase<T>::ReadNext(T &data) {
  return false;
}
template<typename T>
bool ReaderBase<T>::IsOk() const {
  return ok_;
}
template<typename T>
bool ReaderBase<T>::ReadUntil(double gpst, T *pdata) {
  if (!ok_) return ok_;
  T data;
  do {
    if (!ReadNext(data)) {
      return false;
    };
  } while (GetTime(data) < gpst);
  if (pdata) *pdata = data;
  return true;
}
template<typename T>
double ReaderBase<T>::GetTime(const T &data) const {
  return 0;
}
template<typename T>
ReaderBase<T>::ReaderBase() = default;

/**
 * IMU读数据类*/
/**
 * 模板特化，IMU数据读取类
 */
class IMUReader : public ReaderBase<ImuData> {
 public:
  explicit IMUReader(const string &filename,
					 IMUFileFormat fmt = IMU_FILE_IMD,
					 IMUFrame frame = IMU_FRAME_FRD,
					 bool increment = true,
					 int rate = 200);
  void SetFrame(IMUFrame frame);
  void SetFormat(IMUFileFormat format);
  void SetIncrement(bool increment);
  void SetRate(int rate);
  bool ReadNext(ImuData &imu) override;
  double GetTime(const ImuData &imu) const override;
  ~IMUReader();
 private:
  IMUFrame frame_;
  IMUFileFormat format_;
  bool increment_;
  double dt;
};
/** 模板特化
 * GNSS读数据类
 */
class GnssReader : public ReaderBase<GnssData> {
 public:
  explicit GnssReader(std::string &filename, GnssFileFormat format = GNSS_TXT_POS_7);
 public:
  bool ReadNext(GnssData &gnss) override;
  double GetTime(const GnssData &gpst) const override;
 private:
  GnssFileFormat format_;
  GnssMode mode_list[7] = {INVALID, RTK_FIX, RTK_FLOAT, SBAS, RTK_DGPS, SPP, PPP};
};

class OdometerReader : public ReaderBase<Velocity> {
 public:
  explicit OdometerReader(const std::string &file_path);
  bool ReadNext(Velocity &vel) override;
  double GetTime(const Velocity &vel) const override;
};

Option loadOptionFromYml(char path[]);

NavOutput loadNavFromYml(char path[]);

int readImu(ifstream &os, ImuData *pimu, IMUFileFormat is_binary);

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt);

#endif //LOOSELYCOUPLE2020_CPP_FILEIO_H
