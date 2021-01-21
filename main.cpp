#include <iostream>
#include <fstream>
#include "ins_core.h"
#include "Eigen/Dense"

using namespace std;

int main() {
    Eigen::Vector3d v{1, 2, 3};
    std::cout << v << std::endl;
    cout << '\t' << v.x() << endl;
    auto m = convert::skew(v);
    std::cout << m << endl;
    Quad q{1, 2, 3, 4};
    q.normalize();

    m = convert::quaternion_to_dcm(q);
    cout << m << endl;
    auto ll = convert::qne_to_lla(q);
    cout << ll.latitude << "\t" << ll.longitude << endl;
    ofstream f_nav(R"("G:\新建文件夹\HL20181111025454_0IMUFIX.txt")",ios::out|ios::binary);
    if (f_nav.fail()) {
        cout << "failed" << f_nav.rdstate();
        if (f_nav.rdstate() == std::ios_base::failbit) {
            double a = 1;
        }
        return -1;
    }
    f_nav << "hello ins" << endl;
    f_nav.close();
    return 0;
}
