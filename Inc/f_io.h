/**
* @file f_io.h in LooselyCouple2020_cpp
* @author rebeater
* @comment file operation
* Create on 2021/1/16 下午3:56
* @version 1.0
**/

//
// Created by rebeater on 2021/1/16.
//

#ifndef LOOSELYCOUPLE2020_CPP_F_IO_H
#define LOOSELYCOUPLE2020_CPP_F_IO_H

#include <iostream>

#include <fstream>

#include "nav_struct.h"

#include <queue>
#include <mutex>
#include <thread>
using namespace std;


ostream &operator<<(ostream &os, ImuData &imu);

ostream &operator<<(ostream &os, NavOutput output);

ifstream &operator>>(ifstream &is, ImuData &imu);

ifstream &operator>>(ifstream &is, GnssData &gnss);

ostream &operator<<(ostream &os, ImuPara imuPara);


class NavWriter {
    queue<std::shared_ptr<NavOutput>> nav_msgs;
    bool flag_stop;
    std::thread th_write;
    std::mutex mtx_nav;
    string file_path;
private:
    void th_write_nav();
    void start();

public:
    explicit NavWriter(string &filepath);

    ~NavWriter();

    void stop();

    void update(NavOutput &out);
};


Option loadOptionFromYml(char path[]);

NavOutput loadNavFromYml(char path[]);

int readImu(ifstream &os, ImuData *pimu, ImuFileFormat is_binary);

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt);

#endif //LOOSELYCOUPLE2020_CPP_F_IO_H
