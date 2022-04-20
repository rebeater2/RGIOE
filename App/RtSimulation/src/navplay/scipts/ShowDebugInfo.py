#! /usr/bin/python3
import time
import rospy
from navplay.msg import debug
import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt

plt.ion()

class DynamicUpdate():
    # Suppose we know the x range
    def on_launch(self):
        # Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines1, = self.ax.plot([],[], 'o')
        self.lines2, = self.ax.plot([],[], 'o')
        self.lines3, = self.ax.plot([],[], 'o')
        self.ax.set_autoscaley_on(True)
        # Other stuff

        self.ax.grid()
        ...

    def on_running(self, xdata, ydata):
        # Update data (with the new _and_ the old points)
        # self.lines1.set_xdata(xdata)
        # self.lines1.set_ydata(ydata)
        self.lines1.set_data(xdata,ydata)
        # self.lines1.set_xdata(xdata)
        # self.lines1.set_ydata(ydata[:,1])
        # self.lines1.set_xdata(xdata)
        # self.lines1.set_ydata(ydata[:,2])
        # Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

        #Example
    def __call__(self):
        import numpy as np
        import time
        self.on_launch()
        xdata = []
        ydata = []
        for x in np.arange(0,10,0.5):
            xdata.append(x)
            ydata.append(np.exp(-x**2)+10*np.exp(-(x-7)**2))
            self.on_running(xdata, ydata)
            time.sleep(1)
        return xdata, ydata

times =[] #np.array([])
ydata =[] #np.array([])

def is_int(x):
    return x - int(x) < 0.01

def callback(data):
    if not is_int(data.gpst): return
    global times
    times.append(data.gpst)
    # times = np.append(times, data.gpst)
    global ydata
    ydata.append(data.gb[0])
    # ydata = np.append(ydata, data.gb)
    rospy.loginfo("get a debug info:%f %f,ydata:%s", data.gpst, data.gb[0],str(len(ydata)))


def main():
    rospy.init_node("ShowDebugInfo", anonymous=True)
    rospy.Subscriber("/debug_info", debug, callback=callback)
    d = DynamicUpdate()
    d.on_launch()
    while True:
        global times
        global ydata
        d.on_running(times, ydata)
        time.sleep(1)
    rospy.spin()


if __name__ == '__main__':
    main()
