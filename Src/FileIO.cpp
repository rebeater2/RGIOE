//
// Created by rebeater on 2021/1/16.
//

#include "FileIO.h"
#include <iomanip>
#include <fstream>
#include <navigation_log.h>

ostream &operator<<(ostream &os, ImuData &imu) {
  os << imu.gpst << "   ";
  os << std::fixed << left << setprecision(8) << imu.gyro[0] << "   " << imu.gyro[1] << "   " << imu.gyro[2] << "   ";
  os << std::fixed << left << setprecision(8) << imu.acce[0] << "   " << imu.acce[1] << "   " << imu.acce[2] << "   ";
  return os;
};

ifstream &operator>>(ifstream &is, ImuData &imu) {

#if IMU_FRAME == 0 /*前坐上*/
  is >> imu.gpst;
  is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
  is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
#else /*右前下坐标系*/
  is >> imu.gpst;
  is >> imu.gyro[1] >> imu.gyro[0] >> imu.gyro[2];
  is >> imu.acce[1] >> imu.acce[0] >> imu.acce[2];
  imu.gyro[2] *= (-1.0);
  imu.acce[2] *= (-1.0);
#endif
  return is;
};

ifstream &operator>>(ifstream &is, GnssData &gnss) {
  is >> gnss.week>>gnss.gpst;
  is >> gnss.lat >> gnss.lon >> gnss.height;
  is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
  is >> gnss.hdop >> gnss.ns >> gnss.mode;
  return is;
};

ostream &operator<<(ostream &os, NavOutput output) {
  os << 0 << "  " << setprecision(10) << output.gpst << "   ";
  os << left << setprecision(15) << output.pos[0] / _deg << "   " << output.pos[1] / _deg << "   " << output.pos[2]
	 << "   ";
  os << left << setprecision(6) << output.vn[0] << "   " << output.vn[1] << "   " << output.vn[2] << "   ";
  os << left << setprecision(6) << output.atti[0] / _deg << "   " << output.atti[1] / _deg << "   "
	 << output.atti[2] / _deg << "   ";
  os << left << setprecision(10) << output.gb[0] / _deg * _hour << "   " << output.gb[1] / _deg * _hour << "   "
	 << output.gb[2] / _deg * _hour << "  ";
  os << left << setprecision(10) << output.ab[0] / _deg * _hour << "   " << output.ab[1] / _deg * _hour << "   "
	 << output.ab[2] / _deg * _hour<<' ';
#if KD_IN_KALMAN_FILTER==1
  os << output.kd;
#endif
  return os;
}

ostream &operator<<(ostream &os, ImuPara imuPara) {
  os << "arw= " << setprecision(6) << imuPara.arw << endl;
  os << "arw= " << setprecision(6) << imuPara.vrw << endl;
  os << "gb_std= " << setprecision(6) << imuPara.gb_std[0] << "\t" << imuPara.gb_std[01] << imuPara.gb_std[2] << endl;
  os << "ab_std= " << setprecision(6) << imuPara.ab_std[0] << "\t" << imuPara.ab_std[01] << imuPara.ab_std[2] << endl;
  os << "gb_ini= " << setprecision(6) << imuPara.gb_ini[0] << "\t" << imuPara.gb_ini[01] << imuPara.gb_ini[2] << endl;
  os << "ab_ini= " << setprecision(6) << imuPara.ab_ini[0] << "\t" << imuPara.ab_ini[01] << imuPara.ab_ini[2] << endl;
  os << "gt_corr= " << setprecision(6) << imuPara.at_corr << endl;
  os << "at_corr= " << setprecision(6) << imuPara.gt_corr << endl;
  return os;
}
 istream &operator>>( istream &is,  AuxiliaryData &aux){
  is >> aux.gpst >>aux.velocity>>aux.angular_vel;
  return is;
}
ostream & operator<<(ostream & os,GnssData &gnss){
  os<<fixed<<setprecision(0) <<gnss.week<<" " <<setprecision(5) <<gnss.gpst<<" "
	<<setprecision(12)<<gnss.lat<<" "<<gnss.lon<<" "<<setprecision(3)<<gnss.height<<" "
	<<setprecision(3)<<gnss.pos_std[0]<<" "<<gnss.pos_std[1]<<" "<<gnss.pos_std[2]<<"  " <<gnss.hdop<<" "
	<<fixed<<gnss.ns<<" "<< gnss.mode;
  return os;
}

NavWriter::NavWriter(const std::string &file_path) : file_path(file_path) {
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
  mtx_nav.lock();
  flag_stop = true;
  mtx_nav.unlock();
  th_write.join();
}

void NavWriter::th_write_nav() {
  ofstream f_nav(file_path);
  while (true) {
	while (!nav_msgs.empty()) {
	  mtx_nav.lock();
	  auto p_nav = nav_msgs.front();
	  nav_msgs.pop();
	  mtx_nav.unlock();
	  f_nav << *p_nav << "\n";/* */
	}
	mtx_nav.lock();
	if (flag_stop) {
	  break;
	}
	mtx_nav.unlock();
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
};

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt) {
  static string buffer;

  switch (fmt) {
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
  return os.good();
  /* if (fmt == GNSS_TXT_GGA) {
	 getline(os, buffer);
	 stringstream ss(buffer);
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
   return os.good();*/
};