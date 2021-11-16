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

#ifndef LOOSELYCOUPLE2020_CPP_FILEIO_H
#define LOOSELYCOUPLE2020_CPP_FILEIO_H

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <fstream>
#include <condition_variable>

#include "NavStruct.h"

using namespace std;
#define SEPERATE (' ')


ostream &operator<<(ostream &os,const ImuData &imu);

ostream &operator<<(ostream &os,const NavOutput &output);

ifstream &operator>>(ifstream &is, ImuData &imu);
int ReadImu(istream &is,ImuData &imu,ImuFileFormat fmt);


ifstream &operator>>(ifstream &is, GnssData &gnss);

ostream &operator<<(ostream &os,const ImuPara &imuPara);

ostream &operator<<(ostream &os, const AuxiliaryData &aux);
ostream &operator<<(ostream &os,const GnssData &gnss);
 istream &operator>>( istream &is,  AuxiliaryData &aux);

class NavWriter {
  /*多线程读写*/
  queue<std::shared_ptr<NavOutput>> nav_msgs;

  std::thread th_write;
  std::mutex mtx_nav;
  string file_path;
 private:
  atomic_flag flag_running = ATOMIC_FLAG_INIT;
  void th_write_nav();
  void start();

 public:
  explicit NavWriter(string filepath);

  ~NavWriter();

  void stop();

  void update(const NavOutput &out);
};

Option loadOptionFromYml(char path[]);

NavOutput loadNavFromYml(char path[]);

int readImu(ifstream &os, ImuData *pimu, ImuFileFormat is_binary);

int readGnss(ifstream &os, GnssData *pgnss, GnssFileFormat fmt);

#endif //LOOSELYCOUPLE2020_CPP_FILEIO_H
