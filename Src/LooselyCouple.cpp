/**
* @file LooselyCouple.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment C 语言接口,
* Create on 5/9/21 1:18 PM
* @version 1.0
**/
//
// Created by rebeater on 5/9/21.
//

#include "LooselyCouple.h"
#include <DataFusion.h>

DataFusion *df = nullptr;/*point to DataFusion::Instance()*/

void kalmanInitialize(NavOutput *nav, Option *opt) {
    df = &DataFusion::Instance();
    NavEpoch nav_epoch = makeNavEpoch(*nav, *opt);
    df->Initialize(nav_epoch, *opt);
}

void kalmanUpdate(ImuData *imu) {
    df->TimeUpdate(*imu);
}

void kalmanSetGNSS(GnssData *gnss) {
    df->MeasureUpdatePos(*gnss);
}

double kalmanAlignPos(GnssData *gnss, ImuData *imu) {
    return 1.;
};

int kalmanOutput(NavOutput *nav_output) {
    *nav_output = df->Output();
//    nav_output->gpst = gpst;
    /*  nav_output->pos[0] = df->Output();
      nav_output->pos[1] = kf.sins.pos.j;
      nav_output->pos[2] = kf.sins.pos.k;
      *//*转换成float降低文件大小*//*
    nav_output->vn[0] = (float) kf.sins.vn.i;
    nav_output->vn[1] = (float) kf.sins.vn.j;
    nav_output->vn[2] = (float) kf.sins.vn.k;
    nav_output->atti[0] = (float) kf.sins.att.i;
    nav_output->atti[1] = (float) kf.sins.att.j;
    nav_output->atti[2] = (float) kf.sins.att.k;*/

/*    bias->gyro_bias[0] =;
    bias->gyro_bias[1] = (float) (kf.sins.eb.j / _deg * _hour);
    bias->gyro_bias[2] = (float) (kf.sins.eb.k / _deg * _hour);
    bias->acce_bias[0] = (float) (kf.sins.db.i / _mGal);
    bias->acce_bias[1] = (float) (kf.sins.db.j / _mGal);
    bias->acce_bias[2] = (float) (kf.sins.db.k / _mGal);*/
    return 1;
}
#ifdef USE_YAML
#include "Config.h"
void loadYamlConfig(char *yaml_path,char *imu_path,char *gnss_path,char *out_path,Option *opt){
    Config cfg = Config(yaml_path);
    strcpy(imu_path, cfg.imu_filepath.c_str());
    strcpy(gnss_path,cfg.gnss_filepath.c_str());
    strcpy(out_path,cfg.output_filepath.c_str());
    *opt = cfg.getOption();
};
#endif