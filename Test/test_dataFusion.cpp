//
// Created by rebeater on 2020/12/17.
//

#include <FileIO.h>
#include <Config.h>
#include "DataFusion.h"
#include "navigation_log.h"
#include "nav_struct.h"
#include <Timer.h>
#include <Alignment.h>

#define MULTI_THREAD 1
#include <glog/logging.h>

void navExit(const std::string &info) {
  loge << info;
  abort();
}
/**
 * 移动文件指针到开始时间，需要重载各种运算符
 * @tparam T 格式
 * @param is 文件流
 * @param t 数据存储位置
 * @param gpst 要挪到的时间
 */
template<class T>
void moveFilePoint(ifstream &is, T &t, double gpst) {
  do {
	is >> t;
  } while (t.gpst < gpst and is.good());
  is >> t;
}

int main(int argc, char *argv[]) {
  logInit(argv[0], "../../log/");
  cout << CopyRight;
  if (argc < 2) {
	loge << CopyRight << endl;
	return 1;
  }
  Config cfg(argv[1]);
  logi << "imu path:" << cfg.imu_filepath;
  Option opt = cfg.getOption();

  ImuData imu;
  GnssData gnss;
  NavOutput out;
  AuxiliaryData aux;

/*移动文件指针到指定的开始时间*/
  ifstream f_imu(cfg.imu_filepath);
  moveFilePoint(f_imu, imu, cfg.start_time);
  ifstream f_gnss(cfg.gnss_filepath);
  moveFilePoint<GnssData>(f_gnss, gnss, cfg.start_time);
  ifstream f_odo(cfg.odo_filepath);
  moveFilePoint(f_odo, aux, cfg.start_time);
#if MULTI_THREAD == 1
  NavWriter writer(cfg.output_filepath);
#else
  ofstream f_nav(cfg.output_filepath);
#endif

  /*初始对准*/
  NavEpoch nav;
  if (opt.alignmode == ALIGN_MOVING) {
	AlignMoving align{1.5,opt};
	do {
	  readImu(f_imu, &imu, opt.imu_format);
	  align.Update(imu);
	  if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
		logi << "velocity = " << align.Update(gnss);
		readGnss(f_gnss, &gnss, opt.gnss_format);
	  }
	} while (!align.alignFinished() and f_imu.good() and f_gnss.good());
	if (!align.alignFinished()) {
	  navExit("align failed");
	  return -1;
	}
//	logi << "align finished:" << align.getNavEpoch();
	nav = align.getNavEpoch();
	logi<<nav;
  } else if (opt.alignmode == ALIGN_USE_GIVEN) {
	auto nav_ = cfg.getInitNav();
	nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
  }
  Timer timer;

  DataFusion::Instance().Initialize(nav, opt);
  writer.update(DataFusion::Instance().Output());
  /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
  while ((cfg.end_time <= 0) || (cfg.end_time > 0 && imu.gpst < cfg.end_time)) {
	readImu(f_imu, &imu, opt.imu_format);
	if (!f_imu.good())break;
	DataFusion::Instance().TimeUpdate(imu);
	if (f_gnss.good() and fabs(gnss.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  if (DataFusion::Instance().MeasureUpdatePos(gnss) < 0) {
		LOG_EVERY_N(INFO, 1) << "in outage mode" << gnss.gpst;
	  }
	  LOG_EVERY_N(INFO,100)<<"gnss update:"<<gnss.gpst;
	  readGnss(f_gnss, &gnss, opt.gnss_format);
	}
	if (f_odo.good() and fabs(aux.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdateVel(aux.velocity);
	  f_odo >> aux;
	}
#if MULTI_THREAD == 1
	out = DataFusion::Instance().Output();
	writer.update(out);
#else
	f_nav << DataFusion::Instance().Output() << '\n';
#endif
  }
  logi << "epochs:"<<counter;
#if MULTI_THREAD == 1
  logi << "resolve finished, waiting for writing file";
  logi << "time used:" << timer.elapsed() / 1000.0 << "s";
  writer.stop();
  logi << "write finished\n";
#else
  f_nav.close();
#endif
  f_imu.close();
  f_gnss.close();
  logi << "time used:" << timer.elapsed() / 1000.0 << "s";
  logi << DataFusion::Instance().Output() << endl;
  return 0;
}

