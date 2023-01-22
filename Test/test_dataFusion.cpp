//
// Created by rebeater on 2020/12/17.
//
#include "Define.h"
#include "RgioeDataType.h"
#include "FileIO.h"
#include "../App/AppCommon/Config.h"
#include "DataFusion.h"
#include "../App/AppCommon/NavLog.h"
#include "Timer.h"
#include <Alignment.h>

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
  RgioeOption opt = cfg.getOption();

  ImuData imu;
  GnssData gnss;
  NavOutput out;
  AuxiliaryData aux;

/*移动文件指针到指定的开始时间*/
  ifstream f_imu(cfg.imu_filepath);
  moveFilePoint(f_imu, imu, cfg.start_time);
  ifstream f_gnss(cfg.gnss_filepath);
  moveFilePoint(f_gnss, gnss, cfg.start_time);
  ifstream f_odo(cfg.odo_filepath);
  moveFilePoint(f_odo, aux, cfg.start_time);
  NavWriter writer(cfg.output_filepath);

  /*初始对准*/
  NavEpoch nav;
  if (opt.align_mode == RgioeAlignMode::ALIGN_MOVING) {
	logi << "Align moving mode, wait for GNSS";
	AlignMoving align{1.5, opt};
	do {
	  readImu(f_imu, &imu, cfg.imu_format);
	  align.Update(imu);
	  if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
		logi << gnss.gpst << "\tvelocity = " << align.Update(gnss);
		f_gnss >> gnss;
	  }
	} while (!align.alignFinished() and f_imu.good() and f_gnss.good());
	if (!align.alignFinished()) {
	  navExit("align failed");
	  return -1;
	}
	nav = align.getNavEpoch();
  } else if (opt.align_mode == ALIGN_USE_GIVEN) {
	auto nav_ = cfg.getInitNav();
	nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
  } else {
    loge << "supported align mode" << opt.align_mode;
  }
  Timer timer;
  DataFusion::Instance().Initialize(nav, opt);
  writer.update(DataFusion::Instance().Output());
  logi << "initial PVA:" << DataFusion::Instance().Output();
  /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
  while ((cfg.end_time <= 0) || (cfg.end_time > 0 && imu.gpst < cfg.end_time)) {
	f_imu >> imu;
	if (!f_imu.good())break;
	DataFusion::Instance().TimeUpdate(imu);
	if (f_gnss.good() and fabs(gnss.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdatePos(gnss);
	  f_gnss >> gnss;
	}
	if (f_odo.good() and fabs(aux.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdateVel(aux.velocity);
	  f_odo >> aux;
	}
	out = DataFusion::Instance().Output();
	writer.update(out);
  }
  /*show summary and reports:*/
  double time_resolve = timer.elapsed()/1000.0;
  writer.stop();
  double time_writing = timer.elapsed()/1000.0;
  f_imu.close();
  f_gnss.close();
  f_odo.close();
  logi<<"\n Summary:\n"
  << "All epochs:" << DataFusion::Instance().EpochCounter()<<'\n'
  <<"Time for Computing:" << time_resolve<< "s"<<'\n'
  <<"Resolve Speed:"<<1000*time_resolve/DataFusion::Instance().EpochCounter()*cfg.getOption().d_rate*100<<"ms/100s"<<'\n'
  <<"Time for File Writing:"<<time_writing<<'s'<<'\n'
  <<"Final PVA:"<<DataFusion::Instance().Output();

  return 0;
}

