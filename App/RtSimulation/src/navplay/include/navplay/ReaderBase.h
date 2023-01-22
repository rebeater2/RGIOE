/**
* @file ReaderBase.h in Project
* @author rebeater
* @comment
* Create on 1/10/22 10:16 AM
* @version 1.0
**/

#ifndef ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_READERBASE_H_
#define ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_READERBASE_H_
#include <fstream>
template<typename T>
class ReaderBase {
 protected:
  T dat;
 public:
  explicit ReaderBase();
  ~ReaderBase();
  virtual bool ReadNext(T &data);
  /**
   * 虚函数，返回data所在时间
   * @param data
   * @return 时间（gps time)
   */
  virtual double GetTime() const;
  /**
   * 读取下一帧数据，直到数据的GetTime函数达到gpst,将最后一帧数据保存在data里面
   * @param gpst 目标时间
   * @param data 如果为nullptr则不保存数据
   * @return 是否成功，失败原因：1，文件打开失败，2 文件读取完毕
   */
  bool ReadUntil(double gpst, T *data = nullptr);

  bool IsOk() const;
 protected:
  std::ifstream ifs;
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
  } while (GetTime() < gpst);
  if (pdata) *pdata = data;
  return true;
}
template<typename T>
double ReaderBase<T>::GetTime() const {
  return 0;
}
template<typename T>
ReaderBase<T>::~ReaderBase() {
  if (ifs.is_open()) {
	ifs.close();
  }
}
template<typename T>
ReaderBase<T>::ReaderBase() = default;

#endif //ROS_NAV_REPLAY_SRC_NAVPLAY_INCLUDE_NAVPLAY_READERBASE_H_
