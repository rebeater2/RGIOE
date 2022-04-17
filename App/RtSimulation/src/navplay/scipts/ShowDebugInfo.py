import rospy
from navplay.msg import debug
import matplotlib.pyplot as plt
import numpy as np


hl, = plt.plot([], [])


times=hl.get_xdata
ydata=hl.get_ydata

def callback(data):
    times = np.append(hl.get_xdata(), data.gpst)
    ydata = np.append(hl.get_ydata(), data.gpst)

    rospy.loginfo("get a debug info:%f", data.gpst)


def main():
    rospy.init_node("ShowDebugInfo", anonymous=True)
    rospy.Subscriber("/debug_info", debug, callback=callback)
    rospy.spin()
    while True:
        hl.set_xdata(times)
        hl.set_ydata(ydata)
        plt.draw()



if __name__ == '__main__':
    main()
