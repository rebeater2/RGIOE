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
int kalmanInitialize() {
  df = &DataFusion::Instance();
//  if (!align.alignFinished()) { return -1; }
//  NavEpoch nav_epoch = makeNavEpoch(align.getPva(), default_option);
  df->Initialize(align->getNavEpoch(), default_option);
  return 0;
}
void kalmanUpdate(const ImuData *imu) {
  df->TimeUpdate(*imu);
}

void kalmanSetGNSS(const GnssData *gnss) {
  df->MeasureUpdatePos(*gnss);
}
void getXd(double *xds) {
  for (int i = 0; i < STATE_CNT; i++) {
	xds[i] = (double)df->P(i, i) * 1e9;
  }
}

int kalmanAlignLevel(const ImuData *imu) {
  if (align == nullptr) {
	static AlignMoving s_align{2, default_option};
	align = &s_align;
  }
  align->Update(*imu);
  return align->alignFinished();
}

double kalmanAlignGnss(const GnssData *gnss) {
  if (align == nullptr) {
	static AlignMoving s_align{2, default_option};
	align = &s_align;
  }
  return align->Update(*gnss);
}

void kalmanSetVel(const Velocity *vel) {
  df->MeasureUpdateVel({vel->forward, 0, 0});
}

int kalmanOutput(NavOutput *nav_output) {
  *nav_output = df->Output();
  for (int i = 0; i < 3; i++) {
	nav_output->pos_std[i] =(float) df->P(0 + i, 0 + i);
	nav_output->vn_std[i] =(float) df->P(3 + i, 3 + i);
	nav_output->atti_std[i] =(float) df->P(6 + i, 6 + i);
  }
  return 0;
}

#if USE_YAML == 1

#endif