/**
* @file Outage.h in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 3/9/22 11:45 AM
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_INC_OUTAGE_H_
#define LOOSELYCOUPLE2020_CPP_INC_OUTAGE_H_
#include <vector>
class Outage {
 public:
  std::vector<double> starts;
  float outage;
  bool flag_enable;
 public:
  Outage(float start, float stop, float outage, float step);

  Outage();

  /*tell if gpst is in outage mode */
  bool IsOutage(double gpst);
};
#endif //LOOSELYCOUPLE2020_CPP_INC_OUTAGE_H_
