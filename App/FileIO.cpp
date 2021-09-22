//
// Created by rebeater on 2021/1/16.
//

#include "FileIO.h"
#include <iomanip>
#include <fstream>
#include <utility>
#include <sstream>
#include "fmt/format.h"
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

ifstream &operator>>(ifstream &is, ImuData &imu) {

#if IMU_FRAME == 0 /*前坐上*/
  is >> imu.gpst;
  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
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
  is >> gnss.week >> gnss.gpst;
  is >> gnss.lat >> gnss.lon >> gnss.height;
  is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
  is >> gnss.hdop >> gnss.ns >> gnss.mode;
  gnss.yaw = -1;
  return is;
}

ostream &operator<<(ostream &os, const NavOutput &output) {
  os << fmt::format(
	  "{:4d} {:2f} {:.12f} {:.12f} {:.4f} {:.3f} {:.3f} {:.3f} {:.2f} {:.2f} {:.2f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:d} {:d}",
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
  is >> aux.gpst >> aux.velocity >> aux.angular;
  return is;
}
ostream &operator<<(ostream &os, const GnssData &gnss) {
  os << fixed << setprecision(0) << gnss.week << SEPERATE << setprecision(5) << gnss.gpst << SEPERATE
	 << setprecision(12) << gnss.lat << SEPERATE << gnss.lon << SEPERATE << setprecision(3) << gnss.height << SEPERATE
	 << setprecision(3) << gnss.pos_std[0] << SEPERATE << gnss.pos_std[1] << SEPERATE << gnss.pos_std[2] << SEPERATE
	 << gnss.hdop
	 << SEPERATE
	 << fixed << gnss.ns << SEPERATE << gnss.mode;
  return os;
}

NavWriter::NavWriter(std::string file_path) : file_path(std::move(file_path)) {
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
  std::unique_lock<mutex> lk(mtx_nav);
  while (true) {
	while (!nav_msgs.empty()) {
	  mtx_nav.lock();
	  auto p_nav = nav_msgs.front();
	  nav_msgs.pop();
	  mtx_nav.unlock();
	  f_nav << *p_nav << "\n";/* */
	}
	if (flag_running.test_and_set()) {
	  break;
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
int readImu(ifstream &os, ImuData *pimu, ImuFileFormat fmt) {
  if (fmt == IMU_FORMAT_IMD) {
	os.read(reinterpret_cast<char *>(pimu), sizeof(ImuData));
  } else if (fmt == IMU_FORMAT_IMUTXT) {
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