//
// Created by rebeater on 2020/12/17.
//
#include "Define.h"
#include "NavStruct.h"
#include "FileIO.h"
#include "Config.h"
#define GLOG_OUTPUT 1
#include "DataFusion.h"
#include "NavLog.h"
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
  logInit(argv[0], "./");
  cout << CopyRight;
  if (argc < 2) {
	loge << CopyRight << endl;
	return 1;
  }
  Config cfg;
  cfg.LoadFrom(argv[1]);
  logi << "imu path:" << cfg.imu_config.file_path;
  logi << "gnss path:" << cfg.gnss_config.file_path;
  string s;bool ok;
  ok = cfg.LoadImuPara(s);
  Option opt = cfg.GetOption();
  LOG_IF(ERROR,!ok)<<s;

  ImuData imu;
  NavOutput out;
  AuxiliaryData aux;
  IMUSmooth smooth;

  /*移动文件指针到指定的开始时间*/
  ifstream f_imu(cfg.imu_config.file_path);
  do {
	f_imu >> imu;
	smooth.Update(imu);
  } while (imu.gpst < cfg.start_time);
  auto ave = smooth.getSmoothedIMU();
  logi << "smooth:" << ave;
  for (int i = 0; i < 3; i++) {
	opt.imuPara.gb_ini[i] = (float)ave.gyro[i];
  }

  ifstream f_odo(cfg.odometer_config.file_path);
  moveFilePoint(f_odo, aux, cfg.start_time);
  NavWriter writer(cfg.output_path);
  /*初始对准*/
  NavEpoch nav;

  auto nav_ = cfg.align_config.init_pva;
 WGS84::Instance().Update(nav_.lat * _deg, nav_.height);
  nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */

  Timer timer;
  DataFusion::Instance().Initialize(nav, opt);
  logi << "initial PVA:" << DataFusion::Instance().Output();
  writer.update(DataFusion::Instance().Output());
  assert(f_odo.good());
  assert(f_imu.good());
  /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
  while ((cfg.stop_time <= 0) || (cfg.stop_time > 0 && imu.gpst < cfg.stop_time)) {
	f_imu >> imu;
	if (!f_imu.good())break;
	DataFusion::Instance().TimeUpdate(imu);
	if (f_odo.good() and fabs(aux.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdateVel(aux.velocity);
	  do { f_odo >> aux; } while (aux.gpst < imu.gpst);
	}
	out = DataFusion::Instance().Output();
	writer.update(out);
  }
  /*show summary and reports:*/
  double time_resolve = (double)timer.elapsed() / 1000.0;
  writer.stop();
  double time_writing = (double)timer.elapsed() / 1000.0;
  f_imu.close();
  f_odo.close();
  logi << out.kd;
  logi << "\n\tSummary:\n"
	   << "\tAll epochs:" << DataFusion::Instance().EpochCounter() << '\n'
	   << "\tTime for Computing:" << time_resolve << "s" << '\n'
	   << "\tLast Measure Update:" << aux.gpst << '\n'
	   << "\tTime for File Writing:" << time_writing << 's' << '\n'
	   << "\tFinal PVA:" << DataFusion::Instance().Output();
  return 0;
}

