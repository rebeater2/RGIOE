/**
* @file LooselyCouple.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment C 语言接口, 用于嵌入式实时程序
* Create on 5/9/21 1:18 PM
* @version 1.0
**/


#include "LooselyCouple.h"
#include <DataFusion.h>
#include <Alignment.h>
DataFusion *df = nullptr;/*point to DataFusion::Instance()*/
AlignMoving *align = nullptr;/*point to Alignment*/
#if 1
const ImuPara default_imupara{0.15 * _deg / _sqrt_h, 0.25 / _sqrt_h,
							  -0 * _mGal, 0 * _mGal, -0 * _mGal,
							  0 * _deg / _hour, -0 * _deg / _hour, 0 * _deg / _hour,
							  0, 0, 0,
							  0, 0, 0,
							  3.6 * _mGal, 3.6 * _mGal, 3.6 * _mGal,
							  3 * _deg / _hour, 3 * _deg / _hour, 3 * _deg / _hour,
							  1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
							  1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
							  1 * _hour, 1 * _hour
};
NavOutput init_pva{0, 0, 0, 0};

/*zho*/
Option default_option{
  .imuPara=default_imupara,
  .d_rate = 125,
  .align_mode=AlignMode::ALIGN_USE_GIVEN,
  .align_vel_threshold = 1.4,
  .enable_gnss = 1,
  .lb_gnss={0, 0, 0},
  .gnss_std_scale = 1.0,
  .nhc_enable=1,
  .nhc_std= {0.1,0.1},
  .zupt_enable=1,
  .zupt_std=0.01,
  .zupta_enable = 1,
  .zupta_std=0.01 * _deg,
  .odo_enable = 0,
  .odo_std = 0.1,
  .odo_scale = 1.14,
  .odo_scale_std  = 0,
  .lb_wheel={-0.317, -0.095, 0.03},
  .angle_bv={0, 0, 0},
  .pos_std={1, 1, 1},
  .vel_std={2, 2, 2},
  .atti_std={10 * _deg, 10 * _deg, 10 * _deg},
  .output_project_enable = 0,/*输出投影*/
  .pos_project = {0, 0, 0},/*投影到目标位置*/
  .atti_project = {0, 0, 0},/*投影到目标姿态*/
  .enable_rts = 0
};
#endif
/**
 * 卡尔曼滤波初始化，
 * @warning 必须要在初始对准之后完成，如果初始对准未成功，此函数返回-1,y
 * @return error code 0： OK   -1: fail
 */
int navInitialize(const Option *opt) {
  default_option = *opt;
  if (opt->d_rate == 0) {
    return 3;
  }
  df = &(DataFusion::Instance());
  if (!align) { return 1; }
  if (align->alignFinished()) {
    NavEpoch epoch = align->nav;
    Earth::Instance().Update(epoch.pos[0], epoch.pos[2]);
    df->Initialize(align->getNavEpoch(), *opt);
    default_option = *opt;
    return 0;
  } else
    return 2;
}
void timeUpdate(const ImuData *imu) {
  if (!df) {
    df = &(DataFusion::Instance());
  }
#if 0
  double dt = 1. / default_option.d_rate;
  for (int i = 0; i < 3; i++) {
    imu->acce[i] *= (WGS84::Instance().g * dt);
    imu->gyro[i] *= dt;
  }
#endif
  df->TimeUpdate(*imu);
}

void navSetPos(const double latLon[2], float h, const float std[3]) {
  Mat3d rk = Vec3d{std[0] * std[0], std[1] * std[1], std[2] * std[2]}.asDiagonal();
  df->MeasureUpdatePos({latLon[0]*_deg, latLon[1]*_deg, h}, rk);
}
int navSetHeight(double height){
  return df->MeasureUpdateRelativeHeight(height);
}

void getXd(double *xds) {

}

int navAlignLevel(const ImuData *imu) {
  if (align == nullptr) {
    static AlignMoving s_align{ default_option};
    align = &s_align;
  }
  align->Update(*imu);
  return align->alignFinished();
}

double navAlignGnss(const GnssData *gnss) {
  if (align == nullptr) {
    static AlignMoving s_align{default_option};
    align = &s_align;
  }
  return align->Update(*gnss);
}

void navSetVel(const Velocity *vel) {
  df->MeasureUpdateVel({vel->forward, 0, 0});
}

int navGetResult(NavOutput *pva) {
  *pva = df->Output();
  return 0;
}
int navAlignUseGiven(NavOutput *nav, Option *opt) {
  /*室内启动时，使用给定坐标对准*/
  if (align == nullptr) {
    static AlignMoving s_align{*opt};
    align = &s_align;
  }
  align->nav = makeNavEpoch(*nav, *opt);
  align->flag_level_finished = true;
  align->flag_yaw_finished = true;
  return 0;
}
#if USE_YAML == 1

#endif