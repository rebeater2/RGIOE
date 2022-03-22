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
ImuPara default_imupara{0.0112 * _deg / _sqrt_h, 0.0025 / _sqrt_h,
						-1500 * _mGal, 1000 * _mGal, -3000 * _mGal,
						1000 * _deg / _hour, -200 * _deg / _hour, 283 * _deg / _hour,
						0, 0, 0,
						0, 0, 0,
						100 * _mGal, 100 * _mGal, 100 * _mGal,
						1.83 * _deg / _hour, 1 * _deg / _hour, 1 * _deg / _hour,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1 * _hour, 1 * _hour
};
NavOutput default_pva{0, 0, 0, 0};

/*zho*/
Option default_option{
  .imuPara=default_imupara,
  .init_epoch=default_pva,
  .d_rate = 200,
  .align_mode=AlignMode ::ALIGN_USE_GIVEN,
  .nhc_enable=false,
  .zupt_enable=false,
  .zupta_enable=false,
  .odo_enable=false,
  .zupt_std=0.00001,
  .zupta_std=0.1*_deg,
  .lb_gnss={0,0,0},
  .odo_std = 0.000001,
  .lb_wheel={0,0,0},
  .angle_bv={0,0,0},
  .pos_std={0,0,0},
  .vel_std={0,0,0},
  .atti_std={10*_deg,10*_deg,10*_deg},
  .nhc_std={0.00001,0.00001},
  .kd_init=1.0,
  .kd_std=0.001 ,
};
/**
 * 卡尔曼滤波初始化，
 * @warning 必须要在初始对准之后完成，如果初始对准未成功，此函数返回-1,y
 * @return error code 0： OK   -1: fail
 */
int navInitialize(const Option *opt)  {
  default_option = *opt;
  if(opt->d_rate==0){
    return 3;
  }
  df = &(DataFusion::Instance());
  if(!align){return 1;}
  if (align->alignFinished()){
    df->Initialize(align->getNavEpoch(), *opt);
    return 0;
  }else
    return 2;
}
void timeUpdate(const ImuData *imu) {
  if(!df){
    df = &(DataFusion::Instance());
  }
  df->TimeUpdate(*imu);
}

void navSetPos(const double latLon[2], float h, const float std[3]) {
  Mat3d rk = Vec3d{std[0] * std[0], std[1] * std[1], std[2] * std[2]}.asDiagonal();
  df->MeasureUpdatePos({latLon[0], latLon[1], h}, rk);
}
void getXd(double *xds) {

}

int navAlignLevel(const ImuData *imu) {
  if (align == nullptr) {
	static AlignMoving s_align{2, default_option};
	align = &s_align;
  }
  align->Update(*imu);
  return align->alignFinished();
}

double navAlignGnss(const GnssData *gnss) {
  if (align == nullptr) {
	static AlignMoving s_align{2, default_option};
	align = &s_align;
  }
  return align->Update(*gnss);
}

void navSetVel(const Velocity *vel) {
  df->MeasureUpdateVel({vel->forward, 0, 0});
}

int navGetResult(NavPva *pva) {
  NavOutput nav = df->Output();
  *pva=*((NavPva*)&nav.lat);
  return 0;
}
int navAlignUseGiven(NavOutput *nav,Option *opt) {
  /*室内启动时，使用给定坐标对准*/
  if (align == nullptr) {
    static AlignMoving s_align{2, *opt};
    align = &s_align;
  }
  align->nav= makeNavEpoch(*nav,*opt);
  align->flag_level_finished = true;
  align->flag_yaw_finished = true;
  return 0;
}
#if USE_YAML == 1

#endif