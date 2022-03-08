/**
* @file DataFusionThread.cpp in UiDataFusion
* @author rebeater
* @comment
* Create on 11/27/21 11:28 PM
* @version 1.0
**/
#include "DataFusionThread.h"
#include "DataFusion.h"
#include "Alignment.h"
#include "FileIO.h"
#include "Timer.h"
#include <glog/logging.h>

void DataFusionThread::run() {
  LOG(INFO) << "Start DataFusion Thread,Config:\n" << config.ToStdString();
  Option opt = config.GetOption();
  ImuData imu;
  GnssData gnss;
  NavOutput out;
  Velocity vel;
  Outage outage_cfg{config.outage_config.start, config.outage_config.stop, config.outage_config.outage,
					config.outage_config.step};// = cfg.outage_config();
  IMUSmooth smooth;
  IMUReader imu_reader(config.imu_config.file_path,
					   config.imu_config.format,
					   config.imu_config.frame,
					   true, config.imu_config.d_rate);
  if (!imu_reader.IsOk()) {
	LOG(ERROR) << "No such file:" + config.imu_config.file_path;
	return;
  }
  if (!imu_reader.ReadUntil(config.start_time,&imu)) {
	LOG(ERROR) << "IMU data does NOT reach the start time: " << config.start_time;
	SendLog("IMU data does NOT reach the start time" + QString::number(config.start_time));
	return;
  }
  LOG(INFO) << config.gnss_config.format;
  GnssReader gnss_reader(config.gnss_config.file_path, config.gnss_config.format);
  if (!gnss_reader.IsOk()) {
	LOG(ERROR) << "No such file:" + config.gnss_config.file_path;
  }
  if (!gnss_reader.ReadUntil(imu.gpst,&gnss)) {
	LOG(WARNING) << "GNSS data does NOT reach the start time: " << config.start_time;
	SendLog("GNSS data does NOT reach the start time" + QString::number(config.start_time));
  }
  ReaderBase<Velocity> *podoReader = nullptr;//= new OdometerReader(config.odometer_config.file_path);
  if (config.odometer_config.enable) {
	LOG(INFO) << "Odometer path:" << config.odometer_config.file_path;
	podoReader = new OdometerReader(config.odometer_config.file_path);
	if (!podoReader->ReadUntil(imu.gpst,&vel)) {
	  LOG(ERROR) << "Error odometer data does NOT reach the start time";
	  return;
	}
  }
  NavWriter writer(config.output_path);
  NavEpoch nav;
  if (opt.align_mode == AlignMode::ALIGN_MOVING) {
	LOG(INFO) << "Align moving mode, wait for GNSS";
	AlignMoving align{config.align_config.vel_threshold_for_moving, opt};
	do {
	  imu_reader.ReadNext(imu);
	  align.Update(imu);
	  if (fabs(gnss.gpst - imu.gpst) < 1. / opt.d_rate) {
		align.Update(gnss);
		if (!gnss_reader.ReadNext(gnss)) {
		  LOG(ERROR) << "GNSS read finished,but align not complete!";
		  SendLog("GNSS read finished,but align not complete!");
		  return;
		}
	  }
	} while (!align.alignFinished() and imu_reader.IsOk());
	if (!align.alignFinished()) {
	  LOG(ERROR) << "GNSS read finished,but align not complete!";
	  SendLog("GNSS read finished,but align not complete!");
	  return;
	}
	nav = align.getNavEpoch();
  } else if (opt.align_mode == ALIGN_USE_GIVEN) {
	auto nav_ = config.align_config.init_pva;
	nav = makeNavEpoch(nav_, opt);/* 这是UseGiven模式对准 */
  } else {
	LOG(ERROR) << "supported align mode" << (int)opt.align_mode;
	return;
  }
  SendLog("Align finished");
  Timer timer;
  /*第一步：初始化*/
  DataFusion::Instance().Initialize(nav, opt);
  LOG(INFO) << "initial PVA:" << DataFusion::Instance().Output();
  if (config.odometer_config.enable){
    podoReader->ReadUntil(imu.gpst,&vel);
  }
  /* loop function 1: end time <= 0 or 0  < imu.gpst < end time */
  LOG(INFO) << "from:" << imu.gpst << " to: " << config.stop_time;

  while (((config.stop_time <= 0) || (config.start_time > 0 && imu.gpst < config.stop_time)) && imu_reader.IsOk()) {
	if (!imu_reader.ReadNext(imu))break;
	/*第二步 时间更新*/
	DataFusion::Instance().TimeUpdate(imu);
	smooth.Update(imu);
	/*GNSS更新*/
	if (gnss_reader.IsOk() and fabs(gnss.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  if (!(config.outage_config.enable and outage_cfg.IsOutage(gnss.gpst)))
		DataFusion::Instance().MeasureUpdatePos(gnss);
	  LOG_EVERY_N(INFO, 100) << "GNSS update:" << gnss;
	  gnss_reader.ReadUntil(imu.gpst,&gnss);
	}
	/*里程计更新*/
	if (opt.odo_enable and podoReader->IsOk() and fabs(vel.gpst - imu.gpst) < 1.0 / opt.d_rate) {
	  DataFusion::Instance().MeasureUpdateVel(vel.forward);
	  LOG_EVERY_N(INFO, 50 * 100) << "Odo update:" << vel.forward;
	  podoReader->ReadUntil(imu.gpst,&vel);
	}
	if (opt.zupt_enable and smooth.isStatic()) {
	  /*TODO : this will be implemented soon,...maybe*/
//	  DataFusion::Instance().MeasureUpdateStatic();
	}
	out = DataFusion::Instance().Output();
	writer.update(out);
//	usleep(500);
	/*进度打印*/
	SendStatus((int)(100 * (imu.gpst - config.start_time) / (config.stop_time - config.start_time)));
  }
  LOG(INFO) << "Process finished";
  /*show summary and reports:*/
  double time_resolve = static_cast<double>(timer.elapsed()) / 1000.0;
  writer.stop();
  double time_writing = static_cast<double> (timer.elapsed()) / 1000.0;
  delete podoReader;
  LOG(INFO) << "\n\tSummary:\n"
			<< "\tAll epochs:" << DataFusion::Instance().EpochCounter() << '\n'
			<< "\tTime for Computing:" << time_resolve << "s" << '\n'
			<< "\tTime for File Writing:" << time_writing << 's' << '\n'
			<< "\tFinal PVA:" << DataFusion::Instance().Output() << '\n';
  SendLog("Finished,total time:" + QString::number(time_resolve));
  SendStatus(100);
}
bool DataFusionThread::Stop() {
  return false;
}
DataFusionThread::DataFusionThread(Config cfg, QObject *parent) : config(std::move(cfg)) {
  (void )parent;
}

