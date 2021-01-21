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
#include "nav_struct.h"
using namespace std;

ostream & operator << (ostream & os,ImuData &imu);
ostream & operator << (ostream & os,NavOutput output);
ifstream &operator>>(ifstream &is, ImuData &imu);
ifstream &operator>>(ifstream&is,GnssData &gnss);
ostream & operator << (ostream & os,ImuPara imuPara);
Option loadOptionFromYml(char path[]);
NavOutput loadNavFromYml(char path[]);

int readImu (ifstream & os,ImuData *pimu,bool is_binary);
#endif //LOOSELYCOUPLE2020_CPP_F_IO_H
