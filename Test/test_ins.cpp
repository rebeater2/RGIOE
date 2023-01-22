//
// Created by rebeater on 2020/11/10.
//

#include <InsCore.h>
#include <iostream>
#include <fstream>
#include "FileIO.h"

using namespace std;

int main() {
    cout << "hello ins" << endl;
    ifstream f_imu(R"(/mnt/d/NavData/demo/demo_machanization/Data1.bin)", ios::binary);
    ofstream f_nav(R"(/mnt/d/NavData/demo/demo_machanization/rgioe.nav)", ios::out);
    ImuData imu;
    Vec3d pos = Vec3d{23.137395000003920 * _deg, 113.371364999992025 * _deg, 2.174999376267192};
    Vec3d vn = Vec3d{0.000174232435635, -0.000326994722535, 0.000249493122969};
    Vec3d atti = Vec3d{0.010832866167476 * _deg, -2.142487214797399 * _deg, -75.749842669857927 * _deg};
    NavEpoch nav = makeNavEpoch(91620.005000000004657, pos, vn, atti);
    while(imu.gpst<91620.005){
        f_imu.read((char *) &imu, sizeof(ImuData));
    }
    Ins ins;
    ins.InitializePva(nav,imu);
    while (!f_imu.eof()) {
        f_imu.read((char *) &imu, sizeof(ImuData));
        ins.ForwardMechanization(imu);
        f_nav << ins.Output()<<'\n';
    }
    cout<<ins.Output()<<endl;
    f_imu.close();
    f_nav.close();
	return 0;
}