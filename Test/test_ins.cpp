//
// Created by rebeater on 2020/11/10.
//

#include "../Src/InsCore.h"
#include <iostream>
#include <fstream>
#include "FileIO.h"
#include "Config.h"

using namespace std;

int main(int argc, char **argv) {
    if (argc < 3) {
        cout << "usage: test_ins Data1.bin rgioe.nav\n";
    }
    IMUReader reader(argv[1], IMU_FILE_WHU_BIN, IMU_FRAME_FRD, false, 200);
    if(!reader.IsOk()){
        cout <<"file not found:"<<argv[1];
    }
    ofstream f_nav(argv[2], ios::out);
    RgioeImuData imu;
    Vec3d pos = Vec3d{23.137395000003920 * _deg, 113.371364999992025 * _deg, 2.174999376267192};
    Vec3d vn = Vec3d{0.000174232435635, -0.000326994722535, 0.000249493122969};
    Vec3d atti = Vec3d{0.010832866167476 * _deg, -2.142487214797399 * _deg, -75.749842669857927 * _deg};
    Config config;
    config.LoadFrom(R"(C:\Users\linfe\CLionProjects\RGIOE\yaml\pure_ins.yml)");
    NavOutput nav_ = config.align_config.init_pva;
    NavEpoch nav = makeNavEpoch(nav_,config.GetOption());
//    NavEpoch nav = makeNavEpoch(91620.005000000004657, pos, vn, atti);
//    while(imu.gpst<91620.005){
    if(!reader.ReadUntil(91620.005, &imu)){
        cout << "imu data error";
    };
//    }
    Ins ins;
    cout << "init ok. start integrate...\n";
    ins.InitializePva(nav, imu);
    while (reader.IsOk()) {
        reader.ReadNext(imu);
        ins.ForwardMechanization(imu);
        if (imu.gpst > 95220) { break; }
        f_nav << ins.Output() << '\n';
    }
    cout << ins.Output() << endl;
    f_nav.close();
    return 0;
}