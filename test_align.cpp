//
// Created by rebeater on 5/24/21.
//
#include "Alignment.h"
#include <fstream>
#include <iostream>
#include "FileIO.h"

using namespace std;

int main(int argc, char *argv[]) {
    AlignMoving align_moving{1};
    ImuData imu;
    GnssData gnss;
    ifstream f_imu(R"(/media/rebeater/hd_data2/workspace/raw_data/car/20210114/Rover/IMU/adis16460.imd)", ios::binary);
    ifstream f_gnss(R"(/media/rebeater/hd_data2/workspace/raw_data/car/20210114/Rover/GNSS/mxt906b.txt.gga)", ios::in);
    ofstream f_align(R"(/media/rebeater/hd_data2/workspace/raw_data/car/20210114/Rover/IMU/adis16460.smooth.align.nav)",
                     ios::out);
    string buffer;
    for (int i = 0; i < 5; i++)
        getline(f_gnss, buffer);
    readGnss(f_gnss, &gnss, GNSS_TXT_GGA);

    do {
        f_imu.read(reinterpret_cast<char *>(&imu), sizeof(ImuData));
    } while (imu.gpst < gnss.gpst);
    readGnss(f_gnss, &gnss, GNSS_TXT_GGA);
    while (!f_imu.eof() and f_imu.good() and !align_moving.alignFinished()) {
        f_imu.read(reinterpret_cast<char *>(&imu), sizeof(ImuData));
        align_moving.Update(imu);
        if (fabs(gnss.gpst - imu.gpst) < 1. / 100) {
          cout<<"vel:" << align_moving.Update(gnss)<<'\n';
            readGnss(f_gnss, &gnss, GNSS_TXT_GGA);
        }
        /*  auto newimu = imu_smooth.getSmoothedIMU();
          f_imu_out<<newimu<<imu_smooth.getStd()<<" "<<(int)imu_smooth.isStatic()<<'\n';*/
    }
    cout <<align_moving.alignFinished()<<":"<< align_moving.getNav();

}