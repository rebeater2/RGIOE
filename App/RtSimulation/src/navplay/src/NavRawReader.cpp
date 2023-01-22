/**
* @file NavRawReader.cpp in Project
* @author rebeater
* @comment
* Create on 1/10/22 9:56 AM
* @version 1.0
**/


#include <NavRawReader.h>
#include <glog/logging.h>
#include "ReaderBase.h"

bool NavRawReader::ReadNext(RawDataDef &raw) {
  if(!ok_) return false;
  ifs.read(reinterpret_cast<char *>(&dat),sizeof raw);
  raw = dat;
  ok_ = ifs.good();
  return ok_;
}
double NavRawReader::GetTime() const {
  return dat.gpst;
}
NavRawReader::NavRawReader(const std::string &filename) {
  ifs.open(filename,std::ios::binary);
  ok_ = ifs.good();
  LOG_IF(ERROR,!ok_)<<filename<<" open failed";
}
