/**
* @file DataGenerator.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment Generate Odometer and Height data
* Create on 11/17/21 10:12 AM
* @version 1.0
**/

#include <fstream>
#include "../Src/Convert.h"
#include <fmt/format.h>
#include <iostream>

using namespace std;

ifstream &operator>>(ifstream &is, NavOutput &nav) {
    is >> nav.week >> nav.gpst >> nav.lat >> nav.lon >> nav.height >> nav.vn[0] >> nav.vn[1] >> nav.vn[2]
       >> nav.atti[0] >> nav.atti[1] >> nav.atti[2];
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
    return os;
}

int main(int argc, char *argv[]) {

    if (argc < 2) {
        std::cout << "Usage: DataGenerator xxx.nav";
        return 1;
    }
    ifstream f_nav(argv[1]);
    string s(argv[1]);
    ofstream f_odo(s + ".odotxt");
    if (!f_nav.good()) {
        std::cout << "no such file " << argv[1];
    }
    NavOutput nav;
    int skip = 10;/*降低频率*/
    double abv[] = {3, 2, 4};/*安装角*/
    double scale = 1.4; /*比例因子*/

    Vec3d phi_bv = Vec3d{abv[0] * _deg, abv[1] * _deg, abv[2] * _deg};
    Mat3d Cbv = Convert::euler_to_dcm(-phi_bv);
    int cnt = 0;
    while (!f_nav.eof()) {
        f_nav >> nav;
//	LOG_EVERY_N(INFO,100) << nav;
        Mat3d Cbn = Convert::euler_to_dcm({nav.atti[0] * _deg, nav.atti[1] * _deg, nav.atti[2] * _deg});
        Vec3d vn = {nav.vn[0], nav.vn[1], nav.vn[2]};
        Vec3d vb = Cbn.transpose() * vn;
        Vec3d vv = Cbv * vb;
        if (cnt % skip == 0)
            f_odo << fmt::format("{:.4f} {:.4f} {:.4f} {:.4f}\n",
                                 nav.gpst,
                                 vv[0] / scale,
                                 vv[2],
                                 vv[1]);// nav.gpst << ' ' << vv[0] << ' ' << vv[1] << '\n';
        cnt++;
    }
    f_nav.close();
    f_odo.close();
    return 0;
}
