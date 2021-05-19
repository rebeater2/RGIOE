//
// Created by rebeater on 2021/1/16.
//

#ifndef LOOSELYCOUPLE2020_CPP_CONFIG_H
#define LOOSELYCOUPLE2020_CPP_CONFIG_H

#include<iostream>
#include <yaml-cpp/yaml.h>
#include "nav_struct.h"

using namespace std;

class Config {
private:
    YAML::Node root_node;
public:
    string imu_filepath;
    string imu_para_filepath;
    string gnss_filepath;
    string output_filepath;
    double start_time;
    double end_time;
    double d_rate;

public:
    explicit Config(const string& yml_path);
    Option getOption();
    NavOutput getInitNav();/*initial pos vel atti*/
    ImuPara getImuPara(const string &imu_para_path) const;

};
#endif //LOOSELYCOUPLE2020_CPP_CONFIG_H
