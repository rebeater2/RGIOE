/**
* @file AdiHandle.h in InsCubeDecoder
* @author rebeater
* @comment
* Create on 2/23/22 10:05 AM
* @version 1.0
**/

#ifndef INSCUBEDECODER__ADIHANDLE_H_
#define INSCUBEDECODER__ADIHANDLE_H_
#include "RgioeDataType.h"
#include "RawStruct.h"
#include "HandleBase.h"

namespace InsCube {
class ADIHandleBase : public HandleBase {
 public:
  ADIHandleBase();
  ADIHandleBase(double ka, double kg, std::string device,double rate=100,double g=9.8);
 private:
  double rate;
  double g;

  double adis1646x_ka_g = 0.25e-3;
  double adis1646x_kg = 0.005 * _PI / 180.0;
 protected:
  ImuPara para_;
 public:
  double GetRate() const;
  double GetG() const;
  const ImuPara &GetPara() const;
  void ConvertRaw2D(const ImuRawAdi &raw, ImuData &imu) const;
  void Absolute2Increment(ImuData &imu,bool rfu2frd=false) const;
  bool CheckSum(const ImuRawAdi &raw);
};
class Adis16465Handle : public ADIHandleBase {
 public:
  Adis16465Handle(double rate,double g);
  Adis16465Handle();
};
class Adis16460Handle : public ADIHandleBase {
 public:
  Adis16460Handle();
};
}
#endif //INSCUBEDECODER__ADIHANDLE_H_
