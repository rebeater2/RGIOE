/**
* @file ImuFix.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment IMU时间戳修复
* Create on 1/11/22 11:19 AM
* @version 1.0
**/
#include "FileIO.h"
#include "RgioeMath.h"
#include "Convert.h"
#include "fmt/format.h"
#include "glog/logging.h"

void IMUFix() {
    std::string filename = "/mnt/d/NavData/LPS/20220501/RAW_INDOOR000016.raw.imd";
    int rate = 125;
    double dt = 1.0 / rate;
    IMUReader reader(filename, IMU_FILE_IMD, IMU_FRAME_FRD, true, rate);
    ImuData imu;
    reader.ReadNext(imu);
    ImuData pre = imu;
    ofstream out{filename + ".fix.imd", std::ios::binary};
    while (reader.IsOk()) {
        reader.ReadNext(imu);
        imu.gpst -= 1.0;
        out.write((const char *) &imu, sizeof imu);
    }
}

void IMURotate() {
    double rotate_angle[3] = {75, 0, 0};
    std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.imd.fix.imd";
    std::string out_filename = fmt::format("{}_rotate_{}_{}_{}.imd", filename, rotate_angle[0], rotate_angle[1],
                                           rotate_angle[2]);
    ofstream out{out_filename, std::ios::binary};
    int rate = 125;
    Mat3d Cbc = Convert::euler_to_dcm(Vec3d{rotate_angle[0] * _deg, rotate_angle[1] * _deg, rotate_angle[2] * _deg});
    IMUReader reader(filename, IMU_FILE_IMD, IMU_FRAME_FRD, true, rate);
    ImuData imu;
    reader.ReadNext(imu);
    while (reader.IsOk()) {
        reader.ReadNext(imu);
        imu.gpst -= 1.0;
        Vec3d acce{imu.acce}, gyro{imu.gyro};
        Vec3d temp = Cbc * acce;
        imu.acce[0] = temp.x();
        imu.acce[1] = temp.y();
        imu.acce[2] = temp.z();
        temp = Cbc * gyro;
        imu.gyro[0] = temp.x();
        imu.gyro[1] = temp.y();
        imu.gyro[2] = temp.z();
        out.write((const char *) &imu, sizeof imu);
    }
    out.close();
    LOG(INFO) << "save file to " << out_filename;
}

void OdoFix() {
    std::string filename = "/media/rebeater/hd_data2/workspace/raw_data/car/20201215/Rover/ICM_TD/20201215_3.data.prstxt.veltxt";

    OdometerReader reader(filename);
    Velocity vel;
    reader.ReadNext(vel);
    ofstream out{filename + ".fix.veltxt"};
    while (reader.IsOk()) {
        reader.ReadNext(vel);
        vel.gpst -= 1.0;
        out << fmt::format("{:.5f} {:.5f} {:.5f}\n", vel.gpst, vel.forward, vel.angular);
    }
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(".");
    google::LogToStderr();
    IMUFix();
    return 0;
}

