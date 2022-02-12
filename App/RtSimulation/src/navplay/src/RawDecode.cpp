/**
* @file RawDecode.cpp in Project
* @author rebeater
* @comment
* Create on 1/10/22 3:45 PM
* @version 1.0
**/

#include <RawDecode.h>
inline double KyGyroConvert(int16_t low, int16_t out) {
  return 0.02 * out + ((low >> 15) * 0.01) + ((low >> 14) * 0.01 / 2) + ((low >> 13) * 0.01 / 4)
  + ((low >> 12) * 0.01 / 8) + ((low >> 11) * 0.01 / 16) + ((low >> 10) * 0.01 / 32);
};
void ConvertKyToDouble(const ImuRawKy *raw, ImuData *imu) {
  imu->gyro[0] = KyGyroConvert(raw->x_gyro_low, raw->x_gyro_out) * _deg;
  imu->gyro[1] = KyGyroConvert(raw->y_gyro_low, raw->y_gyro_out) * _deg;
  imu->gyro[2] = KyGyroConvert(raw->z_gyro_low, raw->z_gyro_out) * _deg;
  imu->acce[0] = ((int32_t)(raw->x_acce_low + ((int32_t)raw->x_acce_out << 16u))) / 81920000.0;
  imu->acce[1] = ((int32_t)(raw->y_acce_low + ((int32_t)raw->x_acce_out << 16u))) / 81920000.0;
  imu->acce[2] = ((int32_t)(raw->z_acce_low + ((int32_t)raw->z_acce_out << 16u))) / 81920000.0;
};
int ConvertVelRawToFloat(const VelocityRawDef *raw, Velocity *vel) {
  vel->forward = ((float)((int16_t)(raw->vh_ << 8u) | raw->vl_)) / 1000.0f;
  vel->angular = ((float)((int16_t)(raw->ah_ << 8u) | raw->al_)) / 1000.0f;
  return 0;
}
