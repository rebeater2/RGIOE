import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import yaml

path = "/home/rebeater/CLionProjects/RGIOE/yaml/ins_cube.yml"
config = yaml.safe_load(open(path))

def showBias():
    navpath = config["output-path"]
    # df = pd.read_csv(navpath,sep=' ')
    nav = np.loadtxt(navpath)
    times = nav[:, 1]
    plt.figure()
    plt.plot(times, nav[:, 11:14])
    plt.xlabel("gpst/s")
    plt.ylabel("deg/h")
    plt.legend(["x", "y", "z"])
    plt.title("gyroscope bias")

    plt.figure()
    plt.plot(times, nav[:, 14:17])
    plt.xlabel("gpst/s")
    plt.ylabel("mGal")
    plt.title("acce bias")
    plt.legend(["x", "y", "z"])

    plt.figure()
    plt.plot(times,nav[:,19])
    plt.xlabel("gpst/s")
    plt.title("scale factor of odometer")
    plt.show()



def nav_compare():
    nav1 = np.loadtxt(
        "/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/ADIS16465_01/06/ADI51_220307_122202.raw.post.nav")
    nav2 = np.loadtxt("/media/rebeater/hd_data2/workspace/raw_data/2022/20220307/Reference/cutecom0307_04_482_cpt.nav")
    plt.figure()
    plt.plot(nav1[:, 1], nav1[:, 10])
    plt.plot(nav2[:, 1], nav2[:, 10])
    plt.legend(["nav1","nav2"])
    plt.show()



if __name__ == '__main__':
    showBias()
