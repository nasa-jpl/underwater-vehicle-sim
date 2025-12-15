#!/usr/bin/env python

import argparse

import sys
import rospy
from std_msgs.msg import String
from underwater_vehicle_msgs.msg import VehicleData
from nav_msgs.msg import Odometry
import math
import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.stats import chi2

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import matplotlib.colors
from matplotlib.animation import FuncAnimation

colors = ["b", "g", "r", "c", "m", "y", "k"]

xdata = []
ydata = []
scatters = []
cov_ellipses = []
others = []

fig, ax = plt.subplots()
ax.grid()

def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]


def confidence_ellipse(x_cent,y_cent, cov, ax, theta_num=1e3, mass_level=0.68):

    eig_vec,eig_val,u = np.linalg.svd(cov)
    # Make sure 0th eigenvector has positive x-coordinate
    if eig_vec[0][0] < 0:
        eig_vec[0] *= -1
    semimaj = np.sqrt(eig_val[0])
    semimin = np.sqrt(eig_val[1])
    if mass_level is None:
        multiplier = np.sqrt(2.279)
    else:
        distances = np.linspace(0,20,20001)
        chi2_cdf = chi2.cdf(distances,df=2)
        multiplier = np.sqrt(distances[np.where(np.abs(chi2_cdf-mass_level)==np.abs(chi2_cdf-mass_level).min())[0][0]])
    semimaj *= multiplier
    semimin *= multiplier
    phi = np.arccos(np.dot(eig_vec[0],np.array([1,0])))
    if eig_vec[0][1] < 0 and phi > 0:
        phi *= -1

    # Generate data for ellipse structure
    theta = np.linspace(0,2*np.pi,theta_num)
    r = 1 / np.sqrt((np.cos(theta))**2 + (np.sin(theta))**2)
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    data = np.array([x,y])
    S = np.array([[semimaj,0],[0,semimin]])
    R = np.array([[np.cos(phi),-np.sin(phi)],[np.sin(phi),np.cos(phi)]])
    T = np.dot(R,S)

    data = np.dot(T,data)
    data[0] += x_cent
    data[1] += y_cent

    p, = ax.plot(data[0],data[1],color='r',linestyle='-')
    return p

def callback(data, args):
    if len(xdata[args[0]]) % 300 == 0:
        cov = np.zeros((2,2))
        cov[0,0] = data.pose.covariance[7] #flipped x and y for plotting
        cov[0,1] = data.pose.covariance[1]
        cov[1,0] = data.pose.covariance[6]
        cov[1,1] = data.pose.covariance[0]

        if cov[0,0] != 0 or cov[1,1] != 0:
            cov_ellipses[args[0]].append(confidence_ellipse(data.pose.pose.position.y, data.pose.pose.position.x, cov, ax))
            print(cov)
    xdata[args[0]].append(data.pose.pose.position.x)
    ydata[args[0]].append(data.pose.pose.position.y)

def init():
    ax.set_xlim(-1500,1500)
    ax.set_ylim(-1500,11000)
    return scatters

def update(frame):
    for x, y, s in zip(xdata, ydata, scatters):
        if len(x) > 0:
            s.set_offsets(zip(y, x))

    return scatters + others + [item for sublist in cov_ellipses for item in sublist]

def plot():
    ani = FuncAnimation(fig, update, init_func=init, blit=True)

    circleX = []
    circleY = []
    for i in np.arange(0, 2 * math.pi, 0.01):
        circleX.append(math.cos(i) * 1000 + 10000)
        circleY.append(math.sin(i) * 1000 + 10000)
    circle, = ax.plot(circleX, circleY, color='r')

    line2, = ax.plot([], [], color='b')
    beacon = ax.scatter([10000], [10000], color='r')

    others.append(circle)
    others.append(beacon)

    plt.legend((beacon, circle, line2), ('Beacon', '1 km From Beacon', 'AUV Path'))

    plt.show()
    rospy.spin()

def main():
    parser = argparse.ArgumentParser(description='Plot real-time position information from a ROS topic.')
    parser.add_argument('topics', metavar='T', type=str, nargs='+',
                        help='Topics to listen on for position information')
    args = parser.parse_args()
    args.topics
    rospy.init_node('real_time_map_view', anonymous=True)

    for i, arg in enumerate(args.topics):
        rospy.Subscriber(arg, Odometry, callback, (i, ))
        scatters.append(plt.scatter([], [], linewidth=0, animated=True, c=colors[i]))

        xdata.append([])
        ydata.append([])
        cov_ellipses.append([])
    ax.set_aspect('equal', adjustable='box')
    plot()

if __name__ == '__main__':
    main()
