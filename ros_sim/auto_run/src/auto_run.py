import roslaunch
import rospy
import rosbag
import os
import shutil
import sys, time

import subprocess, shlex

from underwater_vehicle_msgs.msg import LogData
#import data_server.srv

import argparse

import psutil, signal

import rosgraph_msgs
import signal

currentState = "N/A"
stateSub = None
roscore = None
currentTime = 0

def handler(signum, frame):
    stopROSBag()
    time.sleep(5)
    roscore.kill()
    sys.exit(1)

def stateCallback(state):
    global currentState
    global currentTime
    currentState = state.stringArrays[0].data[0]
    currentTime = state.header.stamp.secs

def initializeROS(vehicle_name):
    global stateSub
    global roscore
    global clockSub

    roscore = subprocess.Popen('roscore')
    time.sleep(5)
    rospy.init_node('en_Mapping', anonymous=True)
    stateSub = rospy.Subscriber('/' + vehicle_name + '/behavior_state', LogData, stateCallback)

def initializeROSLaunch():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    return uuid

def getLaunchFiles(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and f.endswith(".launch")]

def stopROSBag():
    command = "rosnode kill my_bag"
    command = shlex.split(command)
    subprocess.Popen(command)

def runLaunchFile(uuid, filename, completeDirectory, endTime):
    global currentTime
    global currentState

    launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])

    #Start launch file
    launch.start()
    time.sleep(5)
    
    sawRunning = False

    while (currentState == 'N/A' or currentState == 'RUNNING') and currentTime < endTime:
        try:
            time.sleep(1)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

    sys.stdout.flush()

    # stop the bag recording
    stopROSBag()
    time.sleep(5)
    
    #Shutdown the launch file
    rospy.loginfo("Stopping launch file: %s", filename)
    launch.shutdown()

    #Move completed launch file to completed directory
    if os.path.exists(completeDirectory):
        shutil.move(filename, completeDirectory)

def main(args):
    global currentState

    signal.signal(signal.SIGINT, handler)

    initializeROS(args.vehicle)
    roslaunch_uuid = initializeROSLaunch()

    launchFiles = None

    if not os.path.isfile(args.input):
        if not os.path.exists(os.path.join(args.input, "completed")):
            os.makedirs(os.path.join(args.input, "completed"))
        launchFiles = getLaunchFiles(args.input)
    else:
        launchFiles = [args.input]

    print("Launch Files to Run:")
    for l in launchFiles:
        print(l)

    for launchFile in launchFiles:
        time.sleep(5)
        currentState = 'N/A'
        runLaunchFile(roslaunch_uuid, launchFile, os.path.join(args.input, "completed"), args.end_time)

    roscore.kill()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("input", help="path of launch file or directory containg launch files")
    parser.add_argument('-v', '--vehicle',  help="vehicle to track behavior of", nargs='?', default="v1")
    parser.add_argument('-e', '--end-time',  help="Abort run after this time", nargs='?', default=5011200)
    args = parser.parse_args()

    main(args)