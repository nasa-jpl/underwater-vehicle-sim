import numpy as np
import matplotlib.pyplot as plt
import math
import os
import rosbag
import sys
import xml.etree.ElementTree as ET
import pickle

fixed_range_bias = "0.050000000"
fixed_range_random = "0.050000000"
fixed_ang_vel_bias = "0.000087266"
fixed_ang_vel_random = "0.004363323"

def plot_range_random(data):
    l = {}
    for d in data:
        if d["success"] and d["range_bias_error"] == fixed_range_bias and \
                            d["angular_velocity_random_error"] == fixed_ang_vel_random and \
                            d["angular_velocity_bias_error"][9:-1] == fixed_ang_vel_bias:
            if d["range_random_error"] not in l:
                l[d["range_random_error"]] = []

            l[d["range_random_error"]].append(d["distance_travelled"])
            
    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    proccessed = {}
    avg = []
    success = []
    for k, v in l.items():
        print(k, len(v))
        x = [float(k)] * len(v)
        y = v
        ax1.scatter(x,y, color=color)
        avg.append((float(k),np.mean(v)))
        success.append((float(k),len(v) / 32.0 * 100.0))

    avg.sort()
    success.sort()
    xAvg, yAvg = zip(*avg)
    xSuc, ySuc = zip(*success)

    ax1.plot(xAvg, yAvg)
    ax1.set_ylabel("Average Distance Travelled (m)", color=color)
    ax1.tick_params(axis='y', colors=color)
    ax1.set_ylim((0,86400))

    ax2 = ax1.twinx()

    color = 'tab:red'
    ax2.plot(xSuc, ySuc, color=color)
    ax2.set_ylabel("Percent Successful", color=color)
    ax2.tick_params(axis='y', colors=color)
    ax2.set_ylim((0,101))

    plt.gca().set_xticks([0.01,0.05,0.1,0.2])
    ax1.set_xlabel("Std Dev of Range Gaussian Error (Percent of True Range)")

    plt.show()

def plot_range_bias(data):
    l = {}
    for d in data:
        if d["success"] and d["range_random_error"] == fixed_range_random and \
                            d["angular_velocity_random_error"] == fixed_ang_vel_random and \
                            d["angular_velocity_bias_error"][9:-1] == fixed_ang_vel_bias:
            if d["range_bias_error"] not in l:
                l[d["range_bias_error"]] = []

            l[d["range_bias_error"]].append(d["distance_travelled"])
            
    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    proccessed = {}
    avg = []
    success = []
    for k, v in l.items():
        print(k, len(v))
        x = [float(k)] * len(v)
        y = v
        ax1.scatter(x,y, color=color)
        avg.append((float(k),np.mean(v)))
        success.append((float(k),len(v) / 32.0 * 100.0))

    avg.sort()
    success.sort()
    xAvg, yAvg = zip(*avg)
    xSuc, ySuc = zip(*success)

    ax1.plot(xAvg, yAvg)
    ax1.set_ylabel("Average Distance Travelled (m)", color=color)
    ax1.tick_params(axis='y', colors=color)
    ax1.set_ylim((0,86400))

    ax2 = ax1.twinx()

    color = 'tab:red'
    ax2.plot(xSuc, ySuc, color=color)
    ax2.set_ylabel("Percent Successful", color=color)
    ax2.tick_params(axis='y', colors=color)
    ax2.set_ylim((0,101))

    plt.gca().set_xticks([0.01,0.05,0.1,0.2])
    ax1.set_xlabel("Bias of Range (Percent of True Range)")

    plt.show()

def plot_imu_random(data):
    l = {}
    for d in data:
        if d["launch_file"] == './src/ros-underwater-sim/ros_sim/launch_file_gen/golden_selection_search/params_1_output/golden_selection_homing_run_19_launch/golden_selection_homing_run_19.launch':
            print(d)

        if d["success"] and d["range_bias_error"] == fixed_range_bias and \
                            d["range_random_error"]  == fixed_range_random and \
                            d["angular_velocity_bias_error"][9:-1]  == fixed_ang_vel_bias:
            if d["angular_velocity_random_error"] not in l:
                l[d["angular_velocity_random_error"]] = []

            l[d["angular_velocity_random_error"]].append(d["distance_travelled"])

    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    proccessed = {}
    avg = []
    success = []
    for k, v in l.items():
        print(k, len(v))
        x = [np.degrees(float(k))] * len(v)
        y = v
        ax1.scatter(x,y, color=color)
        avg.append((np.degrees(float(k)),np.mean(v)))
        success.append((np.degrees(float(k)),len(v) / 32.0 * 100.0))

    avg.sort()
    success.sort()
    xAvg, yAvg = zip(*avg)
    xSuc, ySuc = zip(*success)

    ax1.plot(xAvg, yAvg)
    ax1.set_ylabel("Average Distance Travelled (m)", color=color)
    ax1.tick_params(axis='y', colors=color)
    ax1.set_ylim((0,86400))
    ax2 = ax1.twinx()

    color = 'tab:red'
    ax2.plot(xSuc, ySuc, color=color)
    ax2.set_ylabel("Percent Successful", color=color)
    ax2.tick_params(axis='y', colors=color)
    ax2.set_ylim((0,101))

    ax1.set_xticks([0.1,0.25,0.4,1.0, 2.0])
    ax2.set_xticks([0.1,0.25,0.4,1.0, 2.0])

    ax1.set_xlabel("Std Dev of Angular Velocity Gaussian Error (degrees/s)")

    plt.show()

def plot_imu_bias(data):
    l = {}
    for d in data:
        if d["success"] and d["range_bias_error"] == fixed_range_bias and \
                            d["range_random_error"]  == fixed_range_random and \
                            d["angular_velocity_random_error"]  == fixed_ang_vel_random:
            if d["angular_velocity_bias_error"][9:-1] not in l:
                l[d["angular_velocity_bias_error"][9:-1]] = []

            l[d["angular_velocity_bias_error"][9:-1]].append(d["distance_travelled"])
            
    fig, ax1 = plt.subplots()
    color = 'tab:blue'
    proccessed = {}
    avg = []
    success = []
    for k, v in l.items():
        print(k, len(v))
        x = [np.degrees(float(k))] * len(v)
        y = v
        ax1.scatter(x,y, color=color)
        avg.append((np.degrees(float(k)),np.mean(v)))
        success.append((np.degrees(float(k)),len(v) / 32.0 * 100.0))

    success.append((0.025,0))

    avg.sort()
    success.sort()
    xAvg, yAvg = zip(*avg)
    xSuc, ySuc = zip(*success)

    ax1.plot(xAvg, yAvg)
    ax1.set_ylabel("Average Distance Travelled (m)", color=color)
    ax1.tick_params(axis='y', colors=color)
    ax1.set_ylim((0,86400))

    ax2 = ax1.twinx()

    color = 'tab:red'
    ax2.plot(xSuc, ySuc, color=color)
    ax2.set_ylabel("Percent Successful", color=color)
    ax2.tick_params(axis='y', colors=color)
    ax2.set_ylim((0,101))

    ax1.set_xticks([0.001, 0.005, 0.01, 0.025])
    ax2.set_xticks([0.001, 0.005, 0.01, 0.025])

    ax1.set_xlabel("Bias of Angular Velocity (degrees/s)")

    plt.show()


if __name__ == "__main__":
    data = []
    for s in sys.argv[1:]:
        f = open(s,"rb")
        data.extend(pickle.load(f))
        f.close()

    #plot_range_random(data)
    #plot_range_bias(data)
    #plot_imu_random(data)
    plot_imu_bias(data)