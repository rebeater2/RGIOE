/**
* @file AdiHandle.cpp in InsCubeDecoder
* @author rebeater
* @comment
* Create on 2/23/22 10:05 AM
* @version 1.0
**/



#include <string>
#include "AdiHandle.h"
namespace InsCube {
/**
 * 输出单位 加速度计:g 陀螺仪 rad/s
 * @param adi : raw data
 * @param imu : double * 7
 */
void ADIHandleBase::ConvertRaw2D(const ImuRawAdi &adi, ImuData &imu) const {
  imu.gyro[0] = adis1646x_kg * adi.gyro[0];
  imu.gyro[1] = adis1646x_kg * adi.gyro[1];
  imu.gyro[2] = adis1646x_kg * adi.gyro[2];
  imu.acce[0] = adis1646x_ka_g * adi.acce[0];
  imu.acce[1] = adis1646x_ka_g * adi.acce[1];
  imu.acce[2] = adis1646x_ka_g * adi.acce[2];
}
bool ADIHandleBase::CheckSum(const ImuRawAdi &raw) {
  uint16_t sum = 0;
  auto *data = (uint8_t *)&raw;
  for (int i = 0; i < 20; i++) {
	sum += data[i];
  }
  return sum == 0;//*(uint16_t *)&data[18];
}
ADIHandleBase::ADIHandleBase() {
  adis1646x_ka_g = 0;
  adis1646x_kg = 0;

}
ADIHandleBase::ADIHandleBase(double ka, double kg, std::string device, double rate, double g)
	: adis1646x_ka_g(ka), adis1646x_kg(kg), rate(rate), g(g), HandleBase(std::move(device)) {

}
void ADIHandleBase::Absolute2Increment(ImuData &imu, bool rfu2frd) const {
  double dt = 1.0 / rate;
  for (double &i : imu.acce) {
	i *= (dt * g);
  }
  for (double &i : imu.gyro) {
	i *= dt;
  }
  if (rfu2frd) {
	std::swap(imu.acce[0], imu.acce[1]);
	imu.acce[2] *= -1;
	std::swap(imu.gyro[0], imu.gyro[1]);
	imu.gyro[2] *= -1;
  }

}
double ADIHandleBase::GetRate() const {
  return rate;
}
double ADIHandleBase::GetG() const {
  return g;
}
const ImuPara &ADIHandleBase::GetPara() const {
  return para_;
}

Adis16465Handle::Adis16465Handle(double rate, double g) :
	ADIHandleBase(0.25e-3, 1.0 / (160.0) * _PI / 180.0, "ADIS16465", rate, g) {
  para_ = ImuPara{
	  .arw = 0.15 * _deg / _sqrt_h,
	  .vrw =0.012 * _deg / _sqrt_h,
	  .ab_ini={0, 0, 0},
	  .gb_ini ={0, 0, 0},
	  .as_ini = {0, 0, 0},
	  .gs_ini = {0, 0, 0},
	  .ab_std = {3.6 * _mGal, 3.6 * _mGal, 3.6 * _mGal},
	  .gb_std = {1 * _deg / _hour, 1 * _deg / _hour, 1 * _deg / _hour},
	  .gs_std = {1000.0 * _ppm, 1000 * _ppm, 1000 * _ppm},
	  .as_std ={1000 * _ppm, 1000 * _ppm, 1000 * _ppm},
	  .at_corr = 2 * _hour,
	  .gt_corr = 2 * _hour,
  };
}
Adis16465Handle::Adis16465Handle() : ADIHandleBase(0.25e-3, 1.0 / (160.0) * _PI / 180.0, "ADIS16465", 125, 9.8015) {

}
Adis16460Handle::Adis16460Handle() : ADIHandleBase(0.25e-3, 0.005 * _PI / 180.0, "ADIS16460") {
  para_ = ImuPara{
	  .arw = 0.15 * _deg / _sqrt_h,
	  .vrw =0.012 * _deg / _sqrt_h,
	  .ab_ini={0, 0, 0},
	  .gb_ini ={0, 0, 0},
	  .as_ini = {0, 0, 0},
	  .gs_ini = {0, 0, 0},
	  .ab_std = {3.6 * _mGal, 3.6 * _mGal, 3.6 * _mGal},
	  .gb_std = {1 * _deg / _hour, 1 * _deg / _hour, 1 * _deg / _hour},
	  .gs_std = {1000.0 * _ppm, 1000 * _ppm, 1000 * _ppm},
	  .as_std ={1000 * _ppm, 1000 * _ppm, 1000 * _ppm},
	  .at_corr = 2 * _hour,
	  .gt_corr = 2 * _hour,
  };
}
}