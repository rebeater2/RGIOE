
#include <stdio.h>
#include "LooselyCouple.h"

/**/
Option default_option  = {0, IMU_FORMAT_IMUTXT, GNSS_TXT_POS_7,
                                  0, 0, 0,
                                  0, 0, 0, 0, 0,
                                  {0, 0, 0},
                                  {0, 0, 0},
                                  {0, 0, 0},
                                  {0, 0, 0},
                                  {0, 0, 0},
                                  {0, 0, 0},
                                  {0,0},
                                  {0,0}};

/**
 * C test
 * @param argc
 * @param argv
 * @return
 */
int main(int argc,char *argv[]) {
    char imu_path[255],gnss_path[255],out_path[255];
    Option opt;
    loadYamlConfig(argv[1],imu_path,gnss_path,out_path,&opt);
    printf("imu:%s\n gnss:%s\n out:%s\n",imu_path,gnss_path,out_path);
    NavOutput nav;
    GnssData gnss;
//    NavEpoch nav_epoch;
    ImuData imu={0,0,0,0};
    NavOutput out={0,0,0,0};
    kalmanInitialize(&nav,&opt);
    kalmanUpdate(&imu);
    kalmanSetGNSS(&gnss);
    kalmanOutput(&out);

}
