import yaml
import numpy as np
import matplotlib.pyplot as plt

path = "/home/rebeater/CLionProjects/RGIOE/yaml/ins_cube.yml"
config = yaml.safe_load(open(path))


# typedef enum  {
#   SENSOR_NULL = 0x0U,
#   SENSOR_IMU = 0x01U,
#   SENSOR_GNSS = 0x02U,
#   SENSOR_ODO = 0x04U,
#   SENSOR_ZUPT = 0x08U,
#   SENSOR_NHC = 0x10U,
#   SENSOR_CAMERA = 0x20U,
#   SENSOR_LIDAR = 0x40U,
#   SENSOR_HEIGHT = 0x80U,
# }SensorType
def show_sensors():
    result_path = config["Output-Config"]["file-path"]
    data = np.loadtxt(result_path)
    sensor_idx = {
        'imu': 0x01,
        "gnss": 0x02,
        "odometer": 0x04,
        "ZUPT": 0x08,
        "NHC": 0x10,
        "press": 0x80,
    }
    imu_index = []
    gnss_index = []
    odo_index = []
    zupt_index = []
    nhc_index = []
    press_index = []
    for i, d in enumerate(data):
        if int(d[18]) & sensor_idx["imu"] > 0:
            imu_index.append(i)
        if int(d[18]) & sensor_idx["gnss"] > 0:
            gnss_index.append(i)
        if int(d[18]) & sensor_idx["odometer"] > 0:
            odo_index.append(i)
        if int(d[18]) & sensor_idx["ZUPT"] > 0:
            zupt_index.append(i)
        if int(d[18]) & sensor_idx["NHC"] > 0:
            nhc_index.append(i)
        if int(d[18]) & sensor_idx["press"] > 0:
            press_index.append(i)
    times = data[:, 1]
    ones = np.ones(data.shape[0], np.int)
    print(ones.shape)
    print(len(imu_index))
    print(len(gnss_index))
    print(len(odo_index))
    print(len(nhc_index))
    print(len(press_index))
    plt.plot(times[imu_index], 1 * ones[imu_index], 'r')
    plt.plot(times[gnss_index], 2 * ones[gnss_index], '.g')
    plt.plot(times[odo_index], 3 * ones[odo_index], '.b')
    plt.plot(times[zupt_index], 4 * ones[zupt_index], '.k')
    plt.plot(times[nhc_index], 5 * ones[nhc_index], '.c')
    plt.plot(times[press_index], 6 * ones[press_index], '.m')
    plt.xlabel("GPS time/s")
    plt.yticks([1,2,3,4,5,6],["IMU","GNSS","Odometer","ZUPT","NHC","Press"])
    plt.show()



if __name__ == '__main__':
    show_sensors()
