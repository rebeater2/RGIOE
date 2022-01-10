//
// Created by rebeater on 2020/12/17.
//
#include "DataFusion.h"
#include "Alignment.h"
#include "NavStruct.h"
#include "FileIO.h"
#include "Config.h"
#include "NavLog.h"
#include "Timer.h"


#include "fmt/format.h"

/*extern int GnssCheck(const GnssData &gnss){
  if (gnss.ns > 60) {
    return 0;
  }
  if (gnss.ns < 15) {//低于5的抛弃
    return 0;
  }
  if (gnss.mode == SPP) {
    return 1;
  }
  if (gnss.mode == RTK_DGPS) {
    return 2;
  } else if (gnss.mode == RTK_FLOAT || gnss.mode == RTK_FIX) {
    return 3;
  } else {
    return 0;
  }
}*/

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
  logInit(argv[0], "./");
  cout << CopyRight;
  if (argc < 2) {
	loge << CopyRight << endl;
	return 1;
  }
  Config cfg;
  cfg.LoadFrom(argv[1]);
  bool ok;
  string error_msg;
  ok = cfg.LoadImuPara(error_msg);
  Option opt = cfg.GetOption();
  LOG_IF(ERROR, !ok) << error_msg;

  logi << "IMU path:" << cfg.imu_config.file_path;
  if (cfg.odometer_config.enable)
	logi << "Odometer path:" << cfg.odometer_config.file_path;
  logi << "IMU rate:" << opt.d_rate;
  logi << " imu para:\n" << opt.imuPara;
  logi << "gnss path:" << cfg.gnss_config.file_path;

  ImuData imu;
  GnssData gnss;
  NavOutput out;
  AuxiliaryData aux;
  Outage outage_cfg{cfg.outage_config.start, cfg.outage_config.stop, cfg.outage_config.outage,
					cfg.outage_config.step};// = cfg.outage_config();
  IMUSmooth smooth;

/*移动文件指针到指定的开始时间*/
  ifstream f_imu(cfg.imu_config.file_path);
  moveFilePoint(f_imu, imu, cfg.start_time);
  ifstream f_gnss(cfg.gnss_config.file_path);
  moveFilePoint(f_gnss, gnss, cfg.start_time);
  ifstream f_odo(cfg.odometer_config.file_path);
  moveFilePoint(f_odo, aux, cfg.start_time);
  LOG_IF(ERROR, cfg.odometer_config.enable and !f_odo.good()) << "odometer file open failed";
  NavWriter writer(cfg.output_path);
  /*初始对准*/
  NavEpoch nav;
  if (opt.align_mode == AlignMode::ALIGN_MOVING) {
	logi << "Align moving mode, wait for GNSS,threshold is "<<cfg.align_config.vel_threshold_for_moving;
	AlignMoving align{cfg.align_config.vel_threshold_for_moving, opt};
	do {
	  readImu(f_imu, &imu, cfg.imu_config.format);
	  align.Update(imu);
	  if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
		auto res = align.Update(gnss);
		logi <<gnss;
	    logi <<fmt::format("{} vel = {:.3f}",gnss.gpst,res);// gnss.gpst << " velocity = " << res;
		f_gnss >> gnss;
	  }
	} while (!align.alignFinished() and f_imu.good() and f_gnss.good());
	if (!align.alignFinished()) {
//	  navExit("align failed");
	  return 1;
	}
	nav = align.getNavEpoch();
  } else if (opt.align_mode == ALIGN_USE_GIVEN) {
	auto nav_ = cfg.align_config.init_pva;
	nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
  } else {
	logf << "supported align mode" << opt.align_mode;
	return 1;
  }
  Timer timer;
  DataFusion::Instance().Initialize(nav, opt);
  ofstream of_imu(cfg.imu_config.file_path + "smoothed.txt");
  logi << "initial PVA:" << DataFusion::Instance().Output();
  moveFilePoint(f_odo, aux, imu.gpst);

  /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
  while ((cfg.stop_time <= 0) || (cfg.start_time > 0 && imu.gpst < cfg.stop_time)) {
	f_imu >> imu;
	if (!f_imu.good())break;
	DataFusion::Instance().TimeUpdate(imu);
	smooth.Update(imu);
	of_imu << smooth.getSmoothedIMU() << ' ' << smooth.getStd() << ' ' << smooth.isStatic() << '\n';
	if (f_gnss.good() and fabs(gnss.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  if (!outage_cfg.IsOutage(gnss.gpst))
		DataFusion::Instance().MeasureUpdatePos(gnss);
	  LOG_EVERY_N(INFO, 100) << "GNSS update:" << gnss;
	  f_gnss >> gnss;
	}
	LOG_FIRST_N(INFO,10) <<(int) opt.odo_enable <<" "<< f_odo.good() <<" " << fabs(aux.gpst - imu.gpst);
	if (opt.odo_enable and f_odo.good() and fabs(aux.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdateVel(aux.velocity);
	  LOG_EVERY_N(INFO, 50 * 100) << "Odo update:" << aux;
	  do { f_odo >> aux; } while (aux.gpst < imu.gpst and f_odo.good());
	}
	out = DataFusion::Instance().Output();
	writer.update(out);
  }
  /*show summary and reports:*/
  double time_resolve = static_cast<double>(timer.elapsed()) / 1000.0;
  writer.stop();
  double time_writing = static_cast<double> (timer.elapsed()) / 1000.0;
  f_imu.close();
  f_gnss.close();
  f_odo.close();
  logi << "\n\tSummary:\n"
	   << "\tAll epochs:" << DataFusion::Instance().EpochCounter() << '\n'
	   << "\tTime for Computing:" << time_resolve << "s" << '\n'
	   << "\tTime for File Writing:" << time_writing << 's' << '\n'
	   << "\tFinal PVA:" << DataFusion::Instance().Output() << '\n'
	   << "\tFinal Scale Factor:" << out.kd;
  return 0;
}

