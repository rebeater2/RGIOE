/**
* @file Smoother.h in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 3/25/22 6:17 PM
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_INC_SMOOTHER_H_
#define LOOSELYCOUPLE2020_CPP_INC_SMOOTHER_H_
template<typename T>
class Smoother {
 public:
  Smoother(int w = 5);
  Smoother(const T &t, int w = 5);
  ~Smoother();
 T Update(const T &t);
  T GetValue() const ;
 private:
  T data{0};
  int m_window;
  int idx{0};
};
template<typename T>
Smoother<T>::Smoother(int w) :m_window(w) {
}
template<typename T>
Smoother<T>::Smoother(const T &t, int w) : m_window(w), data{std::move(t)} {
};

template<typename T>
T Smoother<T>::Update(const T &t) {
  idx++;
  if (idx < m_window) {
	data = (data * (idx - 1) + t) / idx;
  } else {
	data = (data * (m_window - 1) + t) / m_window;
  }
  return data;
}
template<typename T>
T Smoother<T>::GetValue() const {
  return data;
}
template<typename T>
Smoother<T>::~Smoother<T>() = default;
#endif //LOOSELYCOUPLE2020_CPP_INC_SMOOTHER_H_
