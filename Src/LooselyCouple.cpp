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
AlignMoving *align = nullptr;//(1.5, default_option);
ImuPara default_imupara{0.0112 * _deg / _sqrt_h, 0.0025 / _sqrt_h,
						-1500 * _mGal, 1000 * _mGal, -3000 * _mGal,
						1000 * _deg / _hour, -200 * _deg / _hour, 283 * _deg / _hour,
						0, 0, 0,
						0, 0, 0,
						400 * _mGal, 400 * _mGal, 400 * _mGal,
						4.83 * _deg / _hour, 8 * _deg / _hour, 8 * _deg / _hour,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1000 * _ppm, 1000 * _ppm, 1000 * _ppm,
						1 * _hour, 1 * _hour
};
NavPva default_pva{0, 0, 0, 0};

/*zho*/
Option default_option{
  default_imupara, default_pva,IMU_FORMAT_IMUTXT,GNSS_TXT_POS_7,
	128,
	AlignMode::ALIGN_USE_GIVEN,
	0, 0, 0, 0,
	1, 0.3, 0.01, 0,
	0.1, 0.3, -0.24,
	0.2, 0.35,
	0, 0, 0,
	0, 0, 0,
	0.5, 0.5, 0.9,
	0.2, 0.2, 0.2,
	0.3, 0.3, 0.3,
	0.3, 0.2,
#if KD_IN_KALMAN_FILTER == 1
	1.29, 0.3,
#endif

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
	static AlignMoving s_align{1.5, default_option};
	align = &s_align;
  }
  align->Update(*imu);
  return align->alignFinished();
};

double kalmanAlignGnss(const GnssData *gnss) {
  if (align == nullptr) {
	static AlignMoving s_align{1.5, default_option};
	align = &s_align;
  }
  return align->Update(*gnss);
}

void kalmanSetVel(const Velocity *vel) {
  df->MeasureUpdateVel({vel->forward, 0, 0});
}

int kalmanOutput(NavOutput *nav_output) {
  *nav_output = df->Output();
  return 0;
}

#if USE_YAML == 1
#include "../App/Config.h"
void loadYamlConfig(char *yaml_path, char *imu_path, char *gnss_path, char *out_path, Option *opt, NavOutput *nav) {
	Config cfg = Config(yaml_path);
	strcpy(imu_path, cfg.imu_filepath.c_str());
	strcpy(gnss_path, cfg.gnss_filepath.c_str());
	strcpy(out_path, cfg.output_filepath.c_str());
	*opt = cfg.getOption();
	if (nav != NULL) {
		*nav = cfg.getInitNav();
	}
};
#endif