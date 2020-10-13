import roslaunch
import rospy
import rosbag
import os
import shutil
import sys, time

import subprocess, shlex

from std_msgs.msg import String
#import data_server.srv

import argparse

import psutil, signal

import rosgraph_msgs
import signal



currentStatus = "running"
dataFilePub = None
statusSub = None
clockSub = None
roscore = None
currentTime = 0

def handler(signum, frame):
    stopROSBag()
    roscore.kill()
    sys.exit(1)

def runtimeCallback(data):
    global currentTime

    currentTime = data.clock.secs

def statusCallback(data):
    global currentStatus
    currentStatus = data.data

def initializeROS():
    global statusSub
    global roscore
    global clockSub
    roscore = subprocess.Popen('roscore')
    time.sleep(5)
    rospy.init_node('en_Mapping', anonymous=True)
    statusSub = rospy.Subscriber('/v1/planner_status', String, statusCallback)
    clockSub = rospy.Subscriber('/clock', rosgraph_msgs.msg.Clock, runtimeCallback)


def initializeROSLaunch():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    return uuid

def getLaunchFiles(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and f.endswith(".launch")]

def generateROSBagTopics(vehicles, topic_file, topics):
    bagStr = ""

    if topic_file is not None:
        with open(topic_file, "rt") as fin:
            for line in fin:
                bagStr += line + " "
    if topics is not None:
        for t in topics:
            bagStr += t + " "

    if vehicles is not None:
        if len(vehicles) > 0:
            bagStr += "-e "
        for v in vehicles:
            bagStr += "/" + v + "/(.*) "

    return bagStr

def startROSBag(outputFile, topics):
    command = "rosbag record -O " + outputFile + " " + topics + " __name:=my_bag"
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)

def stopROSBag():
    command = "rosnode kill /my_bag"
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)

def runLaunchFile(uuid, filename, outputFile, completeDirectory, topics):
    global currentTime

    launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])

    #Start ROSBag Recording
    startROSBag(outputFile, topics)

    #Start launch file
    launch.start()
    time.sleep(5)
    
    sawRunning = False

    while (not sawRunning or currentStatus == 'running') and currentTime < 5011200:
        if(currentStatus == 'running'):
            sawRunning = True
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
    

def main(inputs, outputDirectory, topics):
    global currentStatus

    signal.signal(signal.SIGINT, handler)

    initializeROS()
    roslaunch_uuid = initializeROSLaunch()

    launchFiles = None
    outputROSBag = None
    
    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    if not os.path.isfile(inputs):
        if not os.path.exists(os.path.join(inputs, "completed")):
            os.makedirs(os.path.join(inputs, "completed"))

        launchFiles = getLaunchFiles(inputs)
        outputROSBag = [os.path.join(outputDirectory, os.path.basename(file).replace('.launch','.bag')) for file in launchFiles]
    else:
        launchFiles = [inputs]
        outputROSBag = [os.path.join(outputDirectory, os.path.basename(inputs).replace('.launch','.bag'))]

    for launchFile, output in zip(launchFiles, outputROSBag):
        time.sleep(5)
        currentStatus = 'running'
        runLaunchFile(roslaunch_uuid, launchFile, output, os.path.join(inputs, "completed"), topics)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("input", help="path of launch file or directory containg launch files")
    parser.add_argument("output", help="path of directory for output")
    parser.add_argument('-tf', '--topic_file',  help="path to file containing topics to record", default=None)
    parser.add_argument('-v', '--vehicles',  help="vehicle to record all topics for", nargs='*', default=None)
    parser.add_argument('-t', '--topics',  help="topics to record in the rosbag", nargs='*', default=None)

    args = parser.parse_args()

    topics = generateROSBagTopics(args.vehicles, args.topic_file, args.topics)
    main(args.input, args.output, topics)