import numpy as np
import matplotlib.pyplot as plt


def show_debug():
    path = "../build/debug_info.txt"
    Ps = []
    Zs = []
    with open(path, 'r') as f:
        while True:
            line = f.readline()
            if line == "": break
            if line.startswith("P:"):
                data = line.split(":")[1]
                Ps.append([d for d in data.split()])
            if line.startswith("Z:"):
                data = line.split(":")[1]
                Zs.append([d for d in data.split()])
    ps = np.array(Ps, np.double)
    zs = np.array(Zs, np.double)
    # print(ps.shape)
    # print(ps)
    # print(zs.shape)
    plt.figure()
    plt.plot(ps[:, 0])
    plt.title("state vector std")
    plt.figure()
    plt.plot(zs[:, 0])
    plt.title("innovation")
    plt.show()


if __name__ == '__main__':
    show_debug()
