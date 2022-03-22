//
// Created by rebeater on 2021/1/16.
//

#include "FileIO.h"
#include <iomanip>
#include <utility>
#include <sstream>
#include "fmt/format.h"
#include "glog/logging.h"
ostream &operator<<(ostream &os, const ImuData &imu) {
  os << fmt::format("{:.5f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f}",
					imu.gpst,
					imu.gyro[0], imu.gyro[1], imu.gyro[2],
					imu.acce[0], imu.acce[1], imu.acce[2]);// <
  return os;
}
ostream &operator<<(ostream &os, const AuxiliaryData &aux) {
  os << fmt::format("{:.5f} {:8f} {:8f}",
					aux.gpst, aux.velocity, aux.angular);
  return os;
}
/**
 * 返回前右下坐标系的ImuData结构体，其中陀螺单位为rad/s,加速度单位为g
 * @param os
 * @param imu
 * @param fmt
 * @return
 */
int ReadImu(istream &is, ImuData &imu, IMUFileFormat fmt) {
  switch (fmt) {
	case IMU_FILE_IMD: is.read((char *)(&imu), sizeof(imu));
	  for (int i = 0; i < 3; i++) {
		imu.acce[i] *= 200;
		imu.gyro[i] *= 200;
	  }
	  break;
	case IMU_FILE_IMUTXT: {
	  is.read((char *)(&imu), sizeof(imu));
	  double temp = imu.acce[1];
	  imu.acce[1] = imu.acce[0];
	  imu.acce[0] = temp;
	  imu.acce[2] = -imu.acce[2];
	  temp = imu.gyro[1];
	  imu.gyro[1] = imu.gyro[0];
	  imu.gyro[0] = temp;
	  imu.gyro[2] = -imu.gyro[2];
	  break;
	}
/*	case IMU_FORMAT_TXT_FRD: is >> imu.gpst;
	  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
	  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
	  break;
	case IMU_FORMAT_TXT_RFU: {
	  is >> imu.gpst;
	  is >> imu.gyro[1] >> imu.gyro[0] >> imu.gyro[2];
	  is >> imu.acce[1] >> imu.acce[0] >> imu.acce[2];
	  imu.gyro[2] *= (-1.0);
	  imu.acce[2] *= (-1.0);
	  break;
	}*/
	default: break;
  }
  return is.good();
};

ifstream &operator>>(ifstream &is, ImuData &imu) {

#if IMU_FRAME == 0 /**/
  is.read((char *)&imu, sizeof(ImuData));
//  is >> imu.gpst;
//  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
//  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
#else /*zhu右前下坐标系*/
  is >> imu.gpst;
  is >> imu.gyro[1] >> imu.gyro[0] >> imu.gyro[2];
  is >> imu.acce[1] >> imu.acce[0] >> imu.acce[2];
  imu.gyro[2] *= (-1.0);
  imu.acce[2] *= (-1.0);
#endif
  return is;
}

ifstream &operator>>(ifstream &is, GnssData &gnss) {
  is >> gnss.gpst;
  is >> gnss.lat >> gnss.lon >> gnss.height;
  is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
//  is >> gnss.hdop >> gnss.gdop >> gnss.ns >> gnss.mode;
  gnss.yaw = -1;
//  gnss.hdop = 0.001;
//gnss.pos_std[0] *=  0.001;
//  gnss.pos_std[1] *=  0.001;
//  gnss.pos_std[2] *= 0.001;
  return is;
}

ostream &operator<<(ostream &os, const NavOutput &output) {
  os << fmt::format(
	  "{:4d} {:2f} {:.12f} {:.12f} {:.4f} {:10.6f} {:10.6f} {:10.6f} {:8.4f} {:8.4f} {:8.4f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:d} {:d}",
	  output.week,
	  output.gpst,
	  output.lat,
	  output.lon,
	  output.height,
	  output.vn[0],
	  output.vn[1],
	  output.vn[2],
	  output.atti[0],
	  output.atti[1],
	  output.atti[2],
	  output.gb[0],
	  output.gb[1],
	  output.gb[2],
	  output.ab[0],
	  output.ab[1],
	  output.ab[2],
	  output.info.gnss_mode,
	  output.info.sensors
  );

/*  os << output.week << SEPERATE << fixed << setprecision(3) << output.gpst << SEPERATE;
  os << fixed << setprecision(12) << output.lat << SEPERATE << output.lon << SEPERATE;
  os << fixed << setprecision(3) << output.height << SEPERATE;

  os << fixed << setprecision(3) << output.vn[0] << SEPERATE << output.vn[1] << SEPERATE << output.vn[2] << SEPERATE;

  os << fixed << setprecision(3) << output.atti[0]  << SEPERATE << output.atti[1]  << SEPERATE
	 << output.atti[2]  << SEPERATE;

  os << fixed << setprecision(2) << output.gb[0] << SEPERATE << output.gb[1] << SEPERATE
	 << output.gb[2] << SEPERATE;

  os << fixed << setprecision(2) << output.ab[0] << SEPERATE << output.ab[1] << SEPERATE
	 << output.ab[2] << SEPERATE;

  os << output.info.gnss_mode << SEPERATE << output.info.sensors << SEPERATE;
#if KD_IN_KALMAN_FILTER == 1
  os << fixed << setprecision(3) << output.kd << SEPERATE;
#endif*/
  return os;
}

ostream &operator<<(ostream &os, const ImuPara &para) {
  os << fmt::format("arw: {:15f} deg/sqrt_h\n"
					"vrw:{:15f} m/s/sqrt_h\n"
					"gb-std:{:15f} {:15f} {:15f} deg/h\n"
					"ab-std:{:15f} {:15f} {:15f}  mGal\n"
					"gb-ini:{:15f} {:15f} {:15f}  deg/h\n"
					"ab-ini:{:15f} {:15f} {:15f}  mGal\n"
					"acce corr time: {:.1f} h\n"
					"gyro corr time: {:.1f} h\n",
					para.arw / _deg * _sqrt_h,
					para.vrw * _sqrt_h,
					para.gb_std[0] / _deg * _hour, para.gb_std[1] / _deg * _hour, para.gb_std[2] / _deg * _hour,
					para.ab_std[0] / _mGal, para.ab_std[1] / _mGal, para.ab_std[2] / _mGal,
					para.gb_ini[0] / _deg * _hour, para.gb_ini[1] / _deg * _hour, para.gb_ini[2] / _deg * _hour,
					para.ab_ini[0] / _mGal, para.ab_ini[1] / _mGal, para.ab_ini[2] / _mGal,
					para.at_corr / _hour, para.gt_corr / _hour
  );
//  os << "arw= " << setprecision(6) << para.arw << endl;
//  os << "arw= " << setprecision(6) << para.vrw << endl;
//  os << "gb_std= " << setprecision(6) << para.gb_std[0] << "\t" << para.gb_std[01] << para.gb_std[2] << endl;
//  os << "ab_std= " << setprecision(6) << para.ab_std[0] << "\t" << para.ab_std[01] << para.ab_std[2] << endl;
//  os << "gb_ini= " << setprecision(6) << para.gb_ini[0] << "\t" << para.gb_ini[01] << para.gb_ini[2] << endl;
//  os << "ab_ini= " << setprecision(6) << para.ab_ini[0] << "\t" << para.ab_ini[01] << para.ab_ini[2] << endl;
//  os << "gt_corr= " << setprecision(6) << para.at_corr << endl;
//  os << "at_corr= " << setprecision(6) << para.gt_corr << endl;
  return os;
}
istream &operator>>(istream &is, AuxiliaryData &aux) {
  double temp;
  is >> aux.gpst >> aux.velocity >> aux.angular >> temp;
  return is;
}
ostream &operator<<(ostream &os, const GnssData &gnss) {
  os << fixed << setprecision(0) << gnss.week << SEPERATE << setprecision(5) << gnss.gpst << SEPERATE
	 << setprecision(12) << gnss.lat << SEPERATE << gnss.lon << SEPERATE << setprecision(3) << gnss.height << SEPERATE
	 << setprecision(3) << gnss.pos_std[0] << SEPERATE << gnss.pos_std[1] << SEPERATE << gnss.pos_std[2] << SEPERATE
/*	 << gnss.hdop
	 << SEPERATE
	 << fixed << gnss.ns << SEPERATE << gnss.mode*/
	  ;
  return os;
}

NavWriter::NavWriter(std::string file_path) : file_path(std::move(file_path)) {
  flag_running.test_and_set();
  this->start();
}

NavWriter::~NavWriter() {
  if (th_write.joinable())
	th_write.join();
}

void NavWriter::start() {
  thread th(&NavWriter::th_write_nav, this);
  th_write = std::move(th);

}

void NavWriter::update(const NavOutput &out) {
  mtx_nav.lock();
  nav_msgs.emplace(std::make_shared<NavOutput>(out));
  mtx_nav.unlock();
}

void NavWriter::stop() {
  flag_running.clear();
  if (th_write.joinable())
	th_write.join();
}

void NavWriter::th_write_nav() {
  ofstream f_nav(file_path);
  while (flag_running.test_and_set()) {
	while (!nav_msgs.empty()) {
	  mtx_nav.lock();
	  auto p_nav = nav_msgs.front();
	  nav_msgs.pop();
	  mtx_nav.unlock();
	  f_nav << *p_nav <<" "<<p_nav->kd<< "\n";/* */

	}
  }
  f_nav.close();
}
/**
 * binary
 * @param os
 * @param pimu
 * @return
 */
int readImu(ifstream &os, ImuData *pimu, IMUFileFormat fmt) {
  if (fmt == IMU_FILE_IMD) {
	os.read(reinterpret_cast<char *>(pimu), sizeof(ImuData));
  } else if (fmt == IMU_FILE_IMUTXT) {
	os >> (*pimu);
  }
  return os.good();
}

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt) {
  static string buffer;

/*  switch (fmt) {
	case GNSS_TXT_GGA: {
	  getline(os, buffer);
	  stringstream ss(buffer);
	  ss >> pgnss->week >> pgnss->gpst >> pgnss->lat >> pgnss->lon >> pgnss->height >> pgnss->hdop >> pgnss->mode
		 >> pgnss->ns;
	  pgnss->pos_std[0] = 0.1;
	  pgnss->pos_std[1] = 0.1;
	  pgnss->pos_std[2] = 0.1;
	  break;
	}
	case GNSS_TXT_POS_7: os >> (*pgnss);
	  break;
//	case RTKLIB_TXT_POS:break;
	case RESERVED:
	  os >> pgnss->week >> pgnss->gpst >> pgnss->lat >> pgnss->lon >> pgnss->height >> pgnss->pos_std[0]
		 >> pgnss->pos_std[1] >> pgnss->pos_std[2] >> pgnss->hdop >> pgnss->ns >> pgnss->mode;
	  break;
	default:break;
  }
  return os.good();*/
  if (fmt == GNSS_TXT_GGA) {
	getline(os, buffer);
	stringstream ss{buffer};
	ss >> pgnss->week >> pgnss->gpst >> pgnss->lat >> pgnss->lon >> pgnss->height >> pgnss->hdop >> pgnss->mode
	   >> pgnss->ns;
	pgnss->pos_std[0] = 0.1;
	pgnss->pos_std[1] = 0.1;
	pgnss->pos_std[2] = 0.1;
  } else if (fmt == GNSS_TXT_POS_7) {
	os >> (*pgnss);
  } else if (fmt == RTKLIB_TXT_POS) {
	os >> pgnss->week >> pgnss->gpst >> pgnss->lat >> pgnss->lon >> pgnss->height >> pgnss->pos_std[0]
	   >> pgnss->pos_std[1] >> pgnss->pos_std[2];
	pgnss->pos_std[0] = 0.;
	pgnss->pos_std[1] = 0.;
	pgnss->pos_std[2] = 0.;
  }
  return os.good();
}

IMUReader::IMUReader(const string &filename, IMUFileFormat fmt, IMUFrame frame, bool increment, int rate) {
  if (fmt == IMUFileFormat::IMU_FILE_IMD)
	ifs.open(filename, ios::binary);
  else
	ifs.open(filename);
  format_ = fmt;
  frame_ = frame;
  increment_ = increment;
  dt = 1.0 / rate;
  ok_ = ifs.good();
}
IMUReader::~IMUReader() {
  ifs.close();
}

bool IMUReader::ReadNext(ImuData &imu) {
  if (!ok_) { return ok_; }
//LOG(INFO)<<__FILE__<<" "<<__FUNCTION__ <<imu.gpst<<" "<<format_<<" "<<ok_;
  switch (format_) {
	case IMU_FILE_IMD:ifs.read((char *)&imu, sizeof(imu));
	  break;
	case IMU_FILE_IMUTXT:
	  ifs >> imu.gpst >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2] >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
	  break;
	default: ok_ = false;
	  return ok_;
  }
  /*右前上坐标系转换为前右下坐标系*/
  if (frame_ == IMU_FRAME_RFU) {
	swap(imu.acce[0], imu.acce[1]);
	imu.acce[2] *= -1;
	swap(imu.gyro[0], imu.gyro[1]);
	imu.gyro[2] *= -1;
  }
  /*非增量模式数据转换为增量模式数据  @warning: 是否使用相邻两个时刻之间的间隔作为dt更科学呢？*/
/*  if (!increment_) {
	for (int i = 0; i < 3; i++) {
	  imu.acce[i] *= dt;  *//*TODO 重力g没有考虑进去 *//*
	  imu.gyro[i] *= dt;
	}
  }*/

  ok_ = !ifs.eof();
  return ok_;
}
double IMUReader::GetTime(const ImuData &imu) const {
  return imu.gpst;
};
void IMUReader::SetFrame(IMUFrame frame) {
  frame_ = frame;
}
void IMUReader::SetFormat(IMUFileFormat format) {
  format_ = format;
}
void IMUReader::SetIncrement(bool increment) {
  increment_ = increment;
}
void IMUReader::SetRate(int rate) {
  IMUReader::dt = 1.0 / rate;
}

GnssReader::GnssReader(std::string &filename, GnssFileFormat format) {
  ifs.open(filename);
  ok_ = ifs.good();
  format_ = format;
}
bool GnssReader::ReadNext(GnssData &gnss) {
  if (!ok_) { return ok_; }
  string buffer;
  int q;
  getline(ifs, buffer);
  stringstream ss(buffer);
  switch (format_) {
	case GNSS_TXT_POS_7:
	  ss >> gnss.gpst >> gnss.lat >> gnss.lon >> gnss.height >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
	  gnss.mode = GnssMode::SPP;
	  break;
	case RTKLIB_TXT_POS: {
	  ss >> gnss.week >> gnss.gpst >> gnss.lat >> gnss.lon >> gnss.height >> gnss.pos_std[0] >> gnss.pos_std[1]
		 >> gnss.pos_std[2]
		 >> q >> gnss.ns;
/*	  if (q < 0 or q > 7) {
		ok_ = false;
		return ok_;
	  }*/
	  gnss.mode = q;
	}
	  break;
	case GNSS_10_LINES:
	  ss >> gnss.week >> gnss.gpst >> gnss.lat >> gnss.lon >> gnss.height >> gnss.pos_std[0] >> gnss.pos_std[1]
		 >> gnss.pos_std[2] >> gnss.ns >> gnss.mode;// >> gnss.ns;
	  gnss.yaw = -1;
	  gnss.pitch = -1;
	  break;
	default:ok_ = false;
	  return false;
  }
  ok_ = !ifs.eof() and ifs.good();
  return ok_;
}
double GnssReader::GetTime(const GnssData &gnss) const {
  return gnss.gpst;
}

OdometerReader::OdometerReader(const std::string &file_path) {
  ifs.open(file_path);
  ok_ = ifs.good();
}
bool OdometerReader::ReadNext(Velocity &vel) {
  if (!ok_) return ok_;
  std::string buffer;
  getline(ifs, buffer);
  stringstream ss(buffer);
  ss >> vel.gpst >> vel.forward >> vel.angular;
  ok_ = !ifs.eof();
  return ok_;
}
double OdometerReader::GetTime(const Velocity &vel) const {
  return vel.gpst;
}
NavReader::NavReader(string &filename) {
  ifs.open(filename);
  ok_ = ifs.good();
}
bool NavReader::ReadNext(NavOutput &nav) {
  if (!ok_) return ok_;
  std::string buffer;
  getline(ifs, buffer);
  stringstream ss(buffer);
  ss >> nav.week>>nav.gpst >> nav.lat >> nav.lon >> nav.height
	 >> nav.vn[0] >> nav.vn[1] >> nav.vn[2]
	 >> nav.atti[0] >> nav.atti[1] >> nav.atti[2]
	 >> nav.gb[0] >> nav.gb[1] >> nav.gb[2]
	 >> nav.ab[0] >> nav.ab[1] >> nav.ab[2];
  ok_ = !ifs.eof();
  return ok_;
}
double NavReader::GetTime(const NavOutput &nav) const {
  return nav.gpst;
}
BmpReader::BmpReader(const string &filepath) {
  ifs.open(filepath);
  ok_ = ifs.good();
}
double BmpReader::GetTime(const PressureData &press) const {
  return press.gpst;
}
bool BmpReader::ReadNext(PressureData &press) {
  if (!ok_) return ok_;
  std::string buffer;
  getline(ifs, buffer);
  stringstream ss(buffer);
  ss >> press.gpst>>press.pressure;
  ok_ = !ifs.eof();
  return ok_;
}
