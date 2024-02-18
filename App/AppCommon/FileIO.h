/**
* @file FileIO.h in LooselyCouple2020_cpp
* @author rebeater
* @comment file operation
* Create on 2021/1/16 下午3:56
* @version 1.0
**/

#ifndef LOOSELYCOUPLE2020_CPP_FILEIO_H
#define LOOSELYCOUPLE2020_CPP_FILEIO_H

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <fstream>
#include <condition_variable>
#include <rgioe.h>
#include <atomic>

using std::ostream;
using std::istream;
using std::fstream;
using std::ifstream;
using std::ofstream;
using std::atomic_flag;
using std::string;
using std::queue;

#define SEPERATE (' ')
typedef enum {
    GNSS_TXT_POS_7 = 0,
    GNSS_10_LINES = 1,
    GNSS_TXT_POS_14 = 2,
    GNSS_BIN_POS_14 = 3,
    RTKLIB_TXT_POS = 4,
    GNSS_TXT_POS_VEL = 5,
    GNSS_TXT_GGA = 6,
    RESERVED = 6,
} GnssFileFormat;
enum IMUFileFormat {
    IMU_FILE_WHU_BIN = 2,
    IMU_FILE_IMD = 1,
    IMU_FILE_IMUTXT = 0
};
enum NavFileFormat {
    NavBinary = 0,
    NavAscii = 1,
    NavDoubleMatrix = 1,/* discard */
};
struct NavDoubleList {
    double gpst;
    double pos[3];
    double horiz[2];
    double vn[3];
    double atti[3];
    double pos_std[3];
    double vn_std[3];
    double atti_std[3];
};

ostream &operator<<(ostream &os, const RgioeImuData &imu);

ostream &operator<<(ostream &os, const NavOutput &output);

ifstream &operator>>(ifstream &is, RgioeImuData &imu);


ifstream &operator>>(ifstream &is, RgioeGnssData &gnss);

ostream &operator<<(ostream &os, const ImuPara &imuPara);

ostream &operator<<(ostream &os, const AuxiliaryData &aux);

ostream &operator<<(ostream &os, const RgioeGnssData &gnss);

istream &operator>>(istream &is, AuxiliaryData &aux);

/*
 * 单线程写入
 */
class PvaWriter {
public:
    PvaWriter(const std::string &filename, NavFileFormat fmt);
    ~PvaWriter();
    void Write(NavOutput &result);
private:
    std::ofstream ofs;
    NavFileFormat fmt;
};

/**
 * 子线程写入
 */
class NavWriter {

private:
    atomic_flag flag_running = ATOMIC_FLAG_INIT;

    void th_write_nav();

    void start();

    static void ConvertNavToDouble(const NavOutput &nav, NavDoubleList &bin);

public:
    explicit NavWriter(string filepath, NavFileFormat fmt = NavFileFormat::NavAscii);

    ~NavWriter();

    void stop();

    void update(const NavOutput &out);

private:
    /*多线程读写*/
    queue<std::shared_ptr<NavOutput>> nav_msgs;
    std::thread th_write;
    std::mutex mtx_nav;
    string file_path;
    NavFileFormat fmt;
};

typedef double TimeStamp_t;
typedef enum {
    DATA_TYPE_IMU = 1,
    DATA_TYPE_GNSS = 2,
} DataType_t;

class BaseData_t {
public:
    virtual ~BaseData_t() = default;

    TimeStamp_t time;
    DataType_t type;
};

class ImuData_t : public BaseData_t {
public:
    ImuData_t();

    ~ImuData_t() override = default;

    RgioeFloatType acce[3]{};
    RgioeFloatType gyro[3]{};

    RgioeImuData toRgioeData() const;
};


class GnssData_t : public BaseData_t {
public:
    GnssData_t();

    ~GnssData_t() override = default;

    fp64 lat{};
    fp64 lon{};
    float height{};
    float pos_std[3]{};
    float pitch{};
    float yaw{};
    uint32_t mode{};
    uint32_t ns{};
    uint32_t week{};
    fp64 gpst{};

    RgioeGnssData toRgioeData() const;
};

class ReaderBase {
public:
    explicit ReaderBase();

    ~ReaderBase() = default;

    /**exe_20240027_220422exe_20240027_220422
     * 读下一帧数据
     * @return
     */
    virtual std::shared_ptr<BaseData_t> ReadNext() = 0;

    /**
     * 读取下一帧数据，直到数据的GetTime函数达到gpst,将最后一帧数据保存在data里面
     * @param gpst 目标时间
     * @param data 如果为nullptr则不保存数据
     * @return 是否成功，失败原因：1，文件打开失败，2 文件读取完毕
     */
    bool ReadUntil(TimeStamp_t gpst, BaseData_t *data = nullptr);

    bool IsOk() const;

protected:
    ifstream ifs;
    bool ok_;
};


/**
 * IMU读数据类*/
/**
 * 模板特化，IMU数据读取类
 */
class IMUReader : public ReaderBase {
public:
    explicit IMUReader(const string &filename,
                       IMUFileFormat fmt = IMU_FILE_IMD,
                       IMUFrame frame = IMU_FRAME_FRD,
                       bool increment = true,
                       int rate = 200);

    std::shared_ptr<BaseData_t> ReadNext() override;

    ~IMUReader();

private:
    IMUFrame frame_;
    IMUFileFormat format_;
    bool increment_;
    double dt;
};

/** 模板特化
 * GNSS读数据类
 */
class GnssReader : public ReaderBase {
public:
    explicit GnssReader(const std::string &filename, GnssFileFormat format = GNSS_TXT_POS_7);

public:
    std::shared_ptr<BaseData_t> ReadNext() override;

private:
    GnssFileFormat format_;
};
//
//class NavReader : public ReaderBase<NavOutput> {
//public:
//    explicit NavReader(const std::string &filename, NavFileFormat fmt = NavFileFormat::NavBinary);
//
//    explicit NavReader(const char *filename, NavFileFormat fmt = NavFileFormat::NavBinary);
//
//public:
//    bool ReadNext(NavOutput &nav) override;
//
//    double GetTime(const NavOutput &nav) const override;
//
//private:
//    NavFileFormat fmt;
//};
//
//class OdometerReader : public ReaderBase<RgioeOdometerData> {
//public:
//    explicit OdometerReader(const std::string &file_path);
//
//    bool ReadNext(RgioeOdometerData &vel) override;
//
//    double GetTime(const RgioeOdometerData &vel) const override;
//};
//
//class BmpReader : public ReaderBase<PressureData> {
//public:
//    explicit BmpReader(const std::string &filepath);
//
//    bool ReadNext(PressureData &press) override;
//
//    double GetTime(const PressureData &press) const override;
//};

/*
RgioeOption loadOptionFromYml(char path[]);

NavOutput loadNavFromYml(char path[]);*/


#endif //LOOSELYCOUPLE2020_CPP_FILEIO_H
