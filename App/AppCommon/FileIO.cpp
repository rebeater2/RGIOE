//
// Created by rebeater on 2021/1/16.
//

#include "FileIO.h"
#include "../../Src/Earth.h"
#include <utility>
#include <sstream>
#include "fmt/format.h"


ostream &operator<<(ostream &os, const RgioeImuData &imu) {
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

ifstream &operator>>(ifstream &is, RgioeImuData &imu) {

#if IMU_FRAME == 0 /**/
    is.read((char *) &imu, sizeof(RgioeImuData));
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

ifstream &operator>>(ifstream &is, RgioeGnssData &gnss) {
    is >> gnss.gpst;
    is >> gnss.lat >> gnss.lon >> gnss.height;
    is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
    gnss.yaw = -1;
    return is;
}

ostream &operator<<(ostream &os, const NavOutput &output) {
    os << fmt::format(
            "{:4d} {:2f} {:.12f} {:.12f} {:.4f} {:10.6f} {:10.6f} {:10.6f} {:10.6f} {:10.6f} {:10.6f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:8f} {:d} {:d}",
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
            output.gs[0],
            output.gs[1],
            output.gs[2],
            output.as[0],
            output.as[1],
            output.as[2],
            output.info.gnss_mode,
            output.info.sensors
    );
    return os;
}

ostream &operator<<(ostream &os, const ImuPara &para) {
    os << fmt::format("arw: {:15f} deg/sqrt_h\n"
                      "vrw:{:15f} m/s/sqrt_h\n"
                      "gb-std:{:15f} {:15f} {:15f} deg/h\n"
                      "ab-std:{:15f} {:15f} {:15f}  mGal\n"
                      "gs-std:{:15f} {:15f} {:15f} ppm\n"
                      "as-std:{:15f} {:15f} {:15f}  ppm\n"
                      "gb-ini:{:15f} {:15f} {:15f}  deg/h\n"
                      "ab-ini:{:15f} {:15f} {:15f}  mGal\n"
                      "acce corr time: {:.1f} h\n"
                      "gyro corr time: {:.1f} h\n",
                      para.arw / _deg * _sqrt_h,
                      para.vrw * _sqrt_h,
                      para.gb_std[0] / _deg * _hour, para.gb_std[1] / _deg * _hour, para.gb_std[2] / _deg * _hour,
                      para.ab_std[0] / _mGal, para.ab_std[1] / _mGal, para.ab_std[2] / _mGal,
                      para.gs_std[0] / _ppm, para.gs_std[1] / _ppm, para.gs_std[2] / _ppm,
                      para.as_std[0] / _ppm, para.as_std[1] / _ppm, para.as_std[2] / _ppm,
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

ostream &operator<<(ostream &os, const RgioeGnssData &gnss) {
    os << fmt::format("{:.3f} {:.8f} {:.8f}  {:.3f}  {:.3f} {:.3f} {:.3f} {:d} {:d}",
                      gnss.gpst,
                      gnss.lat,
                      gnss.lon,
                      gnss.height,
                      gnss.pos_std[0],
                      gnss.pos_std[1],
                      gnss.pos_std[2],
                      gnss.mode
    );
    return os;
}

NavWriter::NavWriter(std::string file_path, NavFileFormat fmt) : file_path(std::move(file_path)), fmt(fmt) {
    flag_running.test_and_set();
    this->start();
}

NavWriter::~NavWriter() {
    flag_running.clear();
    if (th_write.joinable())
        th_write.join();
}

void NavWriter::start() {
    std::thread th(&NavWriter::th_write_nav, this);
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

void NavWriter::ConvertNavToDouble(const NavOutput &nav, NavDoubleList &bin) {
    static double base_position[3] = {0, 0, 0};
    bin.gpst = nav.gpst;
    bin.pos[0] = nav.lat;
    bin.pos[1] = nav.lon;
    bin.pos[2] = nav.height;
    auto delta_d = Earth::Instance().distance(
            nav.lat * _deg, nav.lon * _deg,
            base_position[0] * _deg, base_position[1] * _deg);
    bin.horiz[0] = delta_d[1];
    bin.horiz[1] = delta_d[0];
    for (int i = 0; i < 3; i++) {
        bin.vn[i] = nav.vn[i];
        bin.atti[i] = nav.atti[i];
        bin.pos_std[i] = nav.pos_std[i];
        bin.atti_std[i] = nav.atti_std[i];
        bin.vn_std[i] = nav.vn_std[i];
    }
}

void NavWriter::th_write_nav() {
    std::ofstream f_nav;
    NavDoubleList ndl;
    if (fmt == NavFileFormat::NavBinary or fmt == NavFileFormat::NavDoubleMatrix)
        f_nav.open(file_path, std::ios::binary);
    else
        f_nav.open(file_path);
    while (flag_running.test_and_set()) {
        while (!nav_msgs.empty()) {
            mtx_nav.lock();
            auto p_nav = nav_msgs.front();
            nav_msgs.pop();
            mtx_nav.unlock();
            if (fmt == NavFileFormat::NavBinary)
                f_nav.write((char *) p_nav.get(), sizeof(NavOutput));
            else if (fmt == NavFileFormat::NavDoubleMatrix) {
                ConvertNavToDouble(*p_nav, ndl);
                f_nav.write((char *) &ndl, sizeof(NavDoubleList));
            } else {
                f_nav << *p_nav << " " << p_nav->kd << "\n";
            }
        }
    }
    f_nav.close();
}

IMUReader::IMUReader(const string &filename, IMUFileFormat fmt, IMUFrame frame, bool increment, int rate) {
    if (fmt == IMUFileFormat::IMU_FILE_IMUTXT)
        ifs.open(filename);
    else
        ifs.open(filename, std::ios::binary);
    format_ = fmt;
    frame_ = frame;
    increment_ = increment;
    dt = 1.0 / rate;
    ok_ = ifs.good();
}

IMUReader::~IMUReader() {
    ifs.close();
}

std::shared_ptr<BaseData_t> IMUReader::ReadNext() {
    std::shared_ptr<ImuData_t> imu = std::make_shared<ImuData_t>();
    imu->type = DataType_t::DATA_TYPE_IMU;
    if (!ok_) { return nullptr; }
    switch (format_) {
        case IMU_FILE_IMD:
            ifs.read((char *) &imu, sizeof(imu));
            for (int i = 0; i < 3; ++i) {
                imu->acce[i] *= (Earth::Instance().g * dt);
                imu->gyro[i] *= dt;
            }
            break;
        case IMU_FILE_IMUTXT:
            ifs >> imu->time >> imu->gyro[0] >> imu->gyro[1] >> imu->gyro[2] >> imu->acce[0] >> imu->acce[1]
                >> imu->acce[2];
            break;
        case IMU_FILE_WHU_BIN:
            fp64 data[7];
            ifs.read((char *) data, sizeof(RgioeImuData));
            for (int i = 0; i < 3; ++i) {
                imu->acce[i] = data[4 + i];
                imu->gyro[i] = data[1 + i];
            }
            imu->time = data[0];
            break;
        default:
            ok_ = false;
            return imu;
    }
    /*右前上坐标系转换为前右下坐标系*/
    if (frame_ == IMU_FRAME_RFU) {
        std::swap(imu->acce[0], imu->acce[1]);
        imu->acce[2] *= -1;
        std::swap(imu->gyro[0], imu->gyro[1]);
        imu->gyro[2] *= -1;
    }
    /*非增量模式数据转换为增量模式数据  @warning: 是否使用相邻两个时刻之间的间隔作为dt更科学呢？*/
/*  if (!increment_) {
	for (int i = 0; i < 3; i++) {
	  imu.acce[i] *= dt;  *//*TODO 重力g没有考虑进去 *//*
	  imu.gyro[i] *= dt;
	}
  }*/
    ok_ = !ifs.eof();
    return imu;
}


GnssReader::GnssReader(const std::string &filename, GnssFileFormat format) {
    ifs.open(filename);
    ok_ = ifs.good();
    format_ = format;
}

std::shared_ptr<BaseData_t> GnssReader::ReadNext() {
    std::shared_ptr<GnssData_t> gnss = std::make_shared<GnssData_t>();
    gnss->type = DataType_t::DATA_TYPE_GNSS;
    if (!ifs.good()) {
        ok_ = false;
        return nullptr;
    }
    string buffer;
    int q;
    std::getline(ifs, buffer);
    std::stringstream ss(buffer);
    switch (format_) {
        case GNSS_TXT_POS_7:
            ss >> gnss->time >> gnss->lat >> gnss->lon >> gnss->height >> gnss->pos_std[0] >> gnss->pos_std[1]
               >> gnss->pos_std[2];
            gnss->mode = GnssMode::SPP;
            gnss->yaw = -1;
            gnss->pitch = -1;
            break;
        case RTKLIB_TXT_POS: {
            {
                int week, ns;
                ss >> week >> gnss->time >> gnss->lat >> gnss->lon >> gnss->height >> gnss->pos_std[0]
                   >> gnss->pos_std[1]
                   >> gnss->pos_std[2]
                   >> q >> ns;
            }
            gnss->mode = q;
        }
            break;
        case GNSS_10_LINES: {
            int week, ns;
            gnss = {0};
            ss >> week >> gnss->time >> gnss->lat >> gnss->lon >> gnss->height >> gnss->pos_std[0] >> gnss->pos_std[1]
               >> gnss->pos_std[2] >> ns >> gnss->mode;// >> gnss->ns;
            gnss->yaw = -1;
            gnss->pitch = -1;
        }
            break;
        case GNSS_TXT_GGA: {
            int week, ns;
            ss >> week >> gnss->time >> gnss->lat >> gnss->lon >> gnss->height >> gnss->pos_std[0] >> gnss->pos_std[1]
               >> gnss->mode >> ns;
            switch (gnss->mode) {
                case RTK_FIX:
                    for (auto &i: gnss->pos_std)
                        i = 0.001;
                    break;
                case RTK_FLOAT:
                    for (auto &i: gnss->pos_std)
                        i = 0.005;
                    break;
                case RTK_DGPS:
                    for (auto &i: gnss->pos_std)
                        i = 1;
                    gnss->mode = INVALID;
                    break;
                case SPP:
                    for (auto &i: gnss->pos_std)
                        i = 1;
                    break;
                default:
                    gnss->mode = INVALID;
                    break;
            }
            gnss->yaw = -1;
            gnss->pitch = -1;
        }
            break;
        default:
            ok_ = false;
    }
    ok_ = !ifs.eof() and ifs.good();
    if (gnss->time > 0 and gnss->mode > 0) return gnss;
    else return nullptr;
}
//
//OdometerReader::OdometerReader(const std::string &file_path) {
//    ifs.open(file_path);
//    ok_ = ifs.good();
//}
//
//bool OdometerReader::ReadNext(RgioeOdometerData &vel) {
//    if (!ok_) return ok_;
//    std::string buffer;
//    getline(ifs, buffer);
//    std::stringstream ss(buffer);
//    ss >> vel.gpst >> vel.forward >> vel.angular;
//    ok_ = !ifs.eof();
//    return ok_;
//}
//
//double OdometerReader::GetTime(const RgioeOdometerData &vel) const {
//    return vel.gpst;
//}
//
//NavReader::NavReader(const string &filename, NavFileFormat fmt) : fmt(fmt) {
//    if (fmt == NavFileFormat::NavBinary) {
//        ifs.open(filename, std::ios::binary);
//    } else
//        ifs.open(filename);
//    ok_ = ifs.good();
//}
//
//NavReader::NavReader(const char *filename, NavFileFormat fmt) : NavReader(std::string(filename), fmt) {
//}
//
//bool NavReader::ReadNext(NavOutput &nav) {
//    if (!ok_) return ok_;
//    if (fmt == NavFileFormat::NavAscii) {
//        std::string buffer;
//        getline(ifs, buffer);
//        std::stringstream ss(buffer);
//        ss >> nav.week >> nav.gpst >> nav.lat >> nav.lon >> nav.height
//           >> nav.vn[0] >> nav.vn[1] >> nav.vn[2]
//           >> nav.atti[0] >> nav.atti[1] >> nav.atti[2]
//           >> nav.gb[0] >> nav.gb[1] >> nav.gb[2]
//           >> nav.ab[0] >> nav.ab[1] >> nav.ab[2];
//    } else {
//        ifs.read((char *) &nav, sizeof(NavOutput));
//    }
//    ok_ = !ifs.eof();
//    return ok_;
//}
//
//double NavReader::GetTime(const NavOutput &nav) const {
//    return nav.gpst;
//}
//
//BmpReader::BmpReader(const string &filepath) {
//    ifs.open(filepath);
//    ok_ = ifs.good();
//}
//
//double BmpReader::GetTime(const PressureData &press) const {
//    return press.gpst;
//}
//
//bool BmpReader::ReadNext(PressureData &press) {
//    if (!ok_) return ok_;
//    std::string buffer;
//    getline(ifs, buffer);
//    std::stringstream ss(buffer);
//    ss >> press.gpst >> press.pressure;
//    ok_ = !ifs.eof();
//    return ok_;
//}
//
//bool ReaderBase::ReadNext(BaseData_t &data) {
//    return false;
//}
//
//ReaderBase::ReaderBase() {
//
//}



bool ReaderBase::ReadUntil(TimeStamp_t gpst, BaseData_t *pdata) {
    if (!ok_) return ok_;
    std::shared_ptr<BaseData_t> data;
    do {
        data = ReadNext();
        if (data == nullptr) {
            return false;
        }
    } while (data->time < gpst);
    if (pdata) *pdata = *data;
    return true;
}

bool ReaderBase::IsOk() const {
    return ok_;
}

ReaderBase::ReaderBase() {

}
