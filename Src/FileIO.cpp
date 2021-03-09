//
// Created by rebeater on 2021/1/16.
//

#include "FileIO.h"
#include <iomanip>
#include <fstream>

ostream &operator<<(ostream &os, ImuData &imu) {
    os << imu.gpst << "   ";
    os << std::fixed << left << setprecision(6) << imu.acce[0] << "   " << imu.acce[1] << "   " << imu.acce[2] << "   ";
    os << std::fixed << left << setprecision(6) << imu.gyro[0] << "   " << imu.gyro[1] << "   " << imu.gyro[2] << "   ";
    return os;
};

ifstream &operator>>(ifstream &is, ImuData &imu) {
    is >> imu.gpst;
    is >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2];
    is >> imu.acce[0] >> imu.acce[1] >> imu.acce[2];
    return is;
};

ifstream &operator>>(ifstream &is, GnssData &gnss) {
    is >> gnss.gpst;
    is >> gnss.lat >> gnss.lon >> gnss.height;
    is >> gnss.pos_std[0] >> gnss.pos_std[1] >> gnss.pos_std[2];
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
       << output.ab[2] / _deg * _hour;
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
NavWriter::NavWriter(std::string &file_path) : file_path(file_path) {
        flag_stop = false;
        this->start();
}

NavWriter::~NavWriter() {
    if (th_write.joinable())
        th_write.join();
}

void NavWriter::start() {
    thread th(&NavWriter::th_write_nav, this);
    th_write = std::move(th);
    flag_stop = false;
}

void NavWriter::update(NavOutput &out) {
    mtx_nav.lock();
    nav_msgs.emplace(std::make_shared<NavOutput>(out));
    mtx_nav.unlock();
}

void NavWriter::stop() {
    flag_stop = true;
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
        if (flag_stop) {
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
};

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt) {
    static string buffer;
    if (fmt == GNSS_TXT_GGA) {
        getline(os, buffer);
        stringstream ss(buffer);
        ss >> pgnss->week >> pgnss->gpst >> pgnss->lat >> pgnss->lon >> pgnss->height >> pgnss->hdop >> pgnss->mode
           >> pgnss->ns;
        pgnss->pos_std[0] = 0.1;
        pgnss->pos_std[1] = 0.1;
        pgnss->pos_std[2] = 0.1;
    } else if (fmt == GNSS_TXT_POS_7) {
        os >> (*pgnss);
    }else if(fmt == RTKLIB_TXT_POS){
        os>>pgnss->week>>pgnss->gpst>>pgnss->lat>>pgnss->lon>>pgnss->height>>pgnss->pos_std[0]>>pgnss->pos_std[1]>>pgnss->pos_std[2];
        pgnss->pos_std[0] = 0.;
        pgnss->pos_std[1] = 0.;
        pgnss->pos_std[2] = 0.;
    }
    return os.good();
};