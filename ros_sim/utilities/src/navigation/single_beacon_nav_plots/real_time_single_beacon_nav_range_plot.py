#!/usr/bin/env python

import sys
import rospy
import tf
from std_msgs.msg import String
from underwater_vehicle_msgs.msg import VehicleData
from underwater_vehicle_msgs.msg import Ranges
from underwater_vehicle_msgs.msg import Range
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

prev_ranges = None
positions = None
last_rot = 0
cov_ellipses = []
fig, ax = plt.subplots()
ax.grid()

def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]


def confidence_ellipse(x_cent,y_cent, cov, theta_num=1e3, mass_level=0.68):

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

    return data[0], data[1]

def odoCallback(data):
    global last_rot
    quaternion = (data.pose.pose.orientation.x,
                  data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z,
                  data.pose.pose.orientation.w)
    last_rot = tf.transformations.euler_from_quaternion(quaternion)[2]

def callback(data):
    global prev_ranges

    if prev_ranges != data.ranges:
        rot = np.zeros((2,2))
        rot[0,0] = math.cos(last_rot)
        rot[0,1] = -math.sin(last_rot)
        rot[1,0] = math.sin(last_rot)
        rot[1,1] = math.cos(last_rot)

        x_pos = []
        y_pos = []
        for i, r in enumerate(data.ranges):
            xy = np.array([[r.x],[r.y]])
            xy = np.matmul(rot,xy)
            x_pos.append(xy[0,0])
            y_pos.append(xy[1,0])

            cov = np.zeros((2,2))
            cov[0,0] = r.position_covariance[0]
            cov[0,1] = r.position_covariance[1]
            cov[1,0] = r.position_covariance[2]
            cov[1,1] = r.position_covariance[3]

            cov = np.matmul(rot, np.matmul(cov, rot.T))

            d_x, d_y = confidence_ellipse(xy[0,0], xy[1,0], cov)
            if len(cov_ellipses) <= i:
                l, = plt.plot(d_y, d_x, c='r')
                cov_ellipses.append(l)
            else:
                cov_ellipses[i].set_data(d_y,d_x)
        if len(x_pos) > 0 and len(y_pos) > 0:
            positions.set_offsets(zip(y_pos, x_pos))
        prev_ranges = data.ranges
def init():
    ax.set_xlim(-1500,1500)
    ax.set_ylim(-1500,1500)
    return [positions] + cov_ellipses

def update(frame):
    #return [positions] + cov_ellipses
    return None

def plot():
    ani = FuncAnimation(fig, update, init_func=init, blit=False)

    plt.show()
    rospy.spin()

def main():
    global positions
    positions = plt.scatter([], [], linewidth=0, animated=True, c='r')

    rospy.init_node('real_time_map_view', anonymous=True)

    rospy.Subscriber(sys.argv[1], Ranges, callback)
    rospy.Subscriber(sys.argv[2], Odometry, odoCallback)

    ax.set_aspect('equal', adjustable='box')
    plot()

if __name__ == '__main__':
    main()
