
#include <stdio.h>

#include "LooselyCouple.h"

/**/

char *str(NavOutput *out) {
    static char buff[1024];
    sprintf(buff, "%f %f %f %f %f %f %f", out->gpst, out->pos[0], out->pos[1], out->pos[2], out->vn[0], out->vn[1],
            out->vn[2]);
    return buff;
}

int main(int argc, char *argv[]) {
    char imu_path[255], gnss_path[255], out_path[255];
    Option opt;
    NavOutput nav;
    GnssData gnss;
    size_t sz;
    loadYamlConfig(argv[1], imu_path, gnss_path, out_path, &opt, &nav);
    printf("imu:%s\n gnss:%s\n out:%s\n", imu_path, gnss_path, out_path);

//    NavEpoch nav_epoch;
    ImuData imu = {0, 0, 0, 0};
    NavOutput out = {0, 0, 0, 0};
    FILE *f_imu = fopen(imu_path, "rb");
    if (f_imu == NULL) {
        printf("no such file : %s\n", imu_path);
        return -1;
    }
    do {
        sz = fread((uint8_t *) &imu, sizeof(double) * 7, 1, f_imu);
    } while (imu.gpst < 456300.0 && sz > 0);

//    fclose(f_imu);
    kalmanInitialize(&nav, &opt);
    kalmanOutput(&out);
    printf("%s\n", str(&out));
    while (sz > 0 && imu.gpst < 459600) {
        sz = fread((uint8_t *) &imu, sizeof(imu), 1, f_imu);

        kalmanUpdate(&imu);
        kalmanOutput(&out);
        printf("%s\n", str(&out));
    }
    printf("%s\n", str(&out));
    fclose(f_imu);
    kalmanSetGNSS(&gnss);
    kalmanOutput(&out);

}
