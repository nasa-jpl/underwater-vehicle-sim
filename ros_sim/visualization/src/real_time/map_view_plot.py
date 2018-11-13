#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from underwater_vehicle_msgs.msg import VehicleData

import numpy as np

import matplotlib.pyplot as plt

import matplotlib.colors
from matplotlib.animation import FuncAnimation

xdata = []
ydata = []
cdata = []
fig, ax = plt.subplots()
ax.grid()
sc = plt.scatter([], [], c=[], linewidth=0, animated=True)

def callback(data):
    xdata.append(data.x)
    ydata.append(data.y)
    cdata.append(data.dye)

def init():
    ax.set_xlim(-50000,50000)
    ax.set_ylim(-50000,50000)
    return sc,

def update(frame):
    if len(xdata) > 0:
        sc.set_offsets(zip(xdata, ydata))
        sc.set_clim(min(0, min(cdata)), max(1, max(cdata)))
        sc.set_array(np.array(cdata).transpose())

    return sc,

def listener(dataStream):

    rospy.init_node('real_time_map_view', anonymous=True)

    rospy.Subscriber(dataStream, VehicleData, callback)

    ani = FuncAnimation(fig, update, init_func=init, blit=True)
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    listener("/vehicles/v1/data_broadcaster/data")

