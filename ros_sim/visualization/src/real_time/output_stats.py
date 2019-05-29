#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from underwater_vehicle_msgs.msg import VehicleData

import math

distanceTravelled = 0
previousXY = None

def callback(data):
    global previousXY
    global distanceTravelled

    if previousXY != None:
        xDiff = data.x - previousXY[0]
        yDiff = data.y - previousXY[1]
        distanceTravelled += math.sqrt(xDiff*xDiff + yDiff*yDiff)

    previousXY = (data.x, data.y)
    print "Distance Travelled: " + str(distanceTravelled) + "\r",

def listener(dataStream):

    rospy.init_node('real_time_map_view', anonymous=True)

    rospy.Subscriber(dataStream, VehicleData, callback)

    rospy.spin()

if __name__ == '__main__':
    listener("/v1/data_broadcaster/data")