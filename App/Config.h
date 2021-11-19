//
// Created by rebeater on 2021/1/16.
//

#ifndef LOOSELYCOUPLE2020_CPP_CONFIG_H
#define LOOSELYCOUPLE2020_CPP_CONFIG_H

#include<iostream>
#include <yaml-cpp/yaml.h>
#include "DataFusion.h"

using namespace std;
/*struct OutageOption{
  int start_time;
  int
};*/

extern Option default_option;
class Config {
 private:
  YAML::Node root_node;
  bool outage_enable;
 public:
  string imu_filepath;
  ImuFileFormat imu_format;
  string imu_para_filepath;
  string gnss_filepath;
  GnssFileFormat gnss_format;
  string odo_filepath;
  string output_filepath;
  double start_time;
  double end_time;
 private:
  Option proc_opt_;
  Outage outage_;
 public:
  explicit Config(const string &yml_path);
  Option getOption () const;
  Outage getOutageConfig()const;
 private:
  Option LoadOption();

  ImuPara LoadImuPara() const;
 public: /*TODO 优化等待*/
  NavOutput getInitNav();/*initial pos vel atti*/
};
#endif //LOOSELYCOUPLE2020_CPP_CONFIG_H
