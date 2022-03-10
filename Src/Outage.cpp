/**
* @file Outage.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 3/9/22 11:45 AM
* @version 1.0
**/

#include "Outage.h"

Outage::Outage(float start, float stop, float outage, float step) : outage(outage) {
  if ((start > stop and stop > 0) or outage < 0 or step < outage) {
    flag_enable = false;
    return;
  }
  flag_enable = true;
  if (stop < 0) stop = start + 4000;
  for (float i = start; i < stop;) {
    starts.push_back(i);
    i += step;
  }
}

/*!
 * 判断当前时间是否处于中断模式
 * */
bool Outage::IsOutage(double gpst) {
  if (!flag_enable) {
    return false;
  }
  for (auto &s:starts) {
    if (gpst <= s + outage) {
      return s <= gpst;
    }
  }
  return false;
}

Outage::Outage() {
  flag_enable = false;
  outage = 0;
}