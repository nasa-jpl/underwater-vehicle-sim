#!/usr/bin/env python

import roslaunch
import rospy
import os
import shutil
import sys

from std_msgs.msg import String
from planner_log.srv import *


strings = []

def callback(data):
    global strings
    strings.append(data.data)

def save(req):

    if not os.path.exists(os.path.dirname(req.filename)):
        os.makedirs(os.path.dirname(req.filename))

    with open(req.filename, 'w+') as f:
        for line in strings:
            f.write(line)
            f.write("\n")

    return SaveLogResponse(True)

def main():

    rospy.init_node('en_Mapping', anonymous=True)
    rospy.Subscriber('planner_log/log', String, callback)
    rospy.Service('planner_log/save', SaveLog, save)
    
    rospy.spin()

if __name__ == "__main__":
    main()