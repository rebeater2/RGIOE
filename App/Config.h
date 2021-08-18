//
// Created by rebeater on 2021/1/16.
//

#ifndef LOOSELYCOUPLE2020_CPP_CONFIG_H
#define LOOSELYCOUPLE2020_CPP_CONFIG_H

#include<iostream>
#include <yaml-cpp/yaml.h>
#include "NavStruct.h"

using namespace std;
extern Option default_option;
class Config {
private:
    YAML::Node root_node;
public:
    string imu_filepath;
    string imu_para_filepath;
    string gnss_filepath;
    string odo_filepath;
    string output_filepath;
    double start_time;
    double end_time;

public:
    explicit Config(const string& yml_path);
    Option getOption();
    NavOutput getInitNav();/*initial pos vel atti*/
    ImuPara getImuPara() const;

};
#endif //LOOSELYCOUPLE2020_CPP_CONFIG_H
