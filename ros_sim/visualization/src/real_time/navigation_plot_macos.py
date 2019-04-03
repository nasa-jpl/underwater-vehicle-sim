#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from underwater_vehicle_msgs.msg import VehicleData
from nav_msgs.msg import Odometry
import math
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import matplotlib.colors
from matplotlib.animation import FuncAnimation

colors = ["b", "g", "r", "c", "m", "y", "k"]

xdata = []
ydata = []
scatters = []

fig, ax = plt.subplots()
ax.grid()

def callback(data, args):
    xdata[args[0]].append(data.pose.pose.position.x)
    ydata[args[0]].append(data.pose.pose.position.y)

def init():
    ax.set_xlim(-50000,50000)
    ax.set_ylim(-50000,50000)
    return scatters

def update(frame):
    for x, y, s in zip(xdata, ydata, scatters):
        if len(x) > 0:
            s.set_offsets(zip(y, x))

    return scatters

def plot():
    ani = FuncAnimation(fig, update, init_func=init, blit=True)
    plt.show()
    rospy.spin()

def main():
    rospy.init_node('real_time_map_view', anonymous=True)

    for i, arg in enumerate(sys.argv[1:]):
        rospy.Subscriber(arg, Odometry, callback, (i, ))
        scatters.append(plt.scatter([], [], linewidth=0, animated=True, c=colors[i]))

        xdata.append([])
        ydata.append([])

    plot()

if __name__ == '__main__':
    main()
