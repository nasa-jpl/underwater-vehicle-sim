import numpy as np
import matplotlib.pyplot as plt
import math
import os
import rosbag
import sys
import xml.etree.ElementTree as ET
import pickle

def findXMLElement(tag, attributes, root):
    for elem in root.findall(".//" + tag):
        correct = True
        for k,v in attributes.items():
            if elem.get(k) != v:
                correct = False
        if correct:
            return elem
    return None


def genTestDataEntry(launchFile, statFile, bagFile):
    testDataEntry = {}

    testDataEntry["success"] = False
    with open(statFile) as file:
        for line in file:
            if "success" in line:
                testDataEntry["success"] = True

    xmlRoot = ET.parse(launchFile).getroot()
    rangeErrorElem = findXMLElement("rosparam", {"param": "usbl/range_random_error"}, xmlRoot)
    rangeBiasElem = findXMLElement("rosparam", {"param": "usbl/range_bias_error"}, xmlRoot)
    angVelRandomError = findXMLElement("rosparam", {"param": "imu/angular_velocity_random_error"}, xmlRoot)
    angVelBiasError = findXMLElement("rosparam", {"param": "imu/angular_velocity_bias_error"}, xmlRoot)

    beaconX = float(findXMLElement("rosparam", {"param": "usbl/beacon_x"}, xmlRoot).text)
    beaconY = float(findXMLElement("rosparam", {"param": "usbl/beacon_y"}, xmlRoot).text)

    testDataEntry["range_random_error"] = rangeErrorElem.text
    testDataEntry["range_bias_error"] = rangeBiasElem.text
    testDataEntry["angular_velocity_random_error"] = angVelRandomError.text
    testDataEntry["angular_velocity_bias_error"] = angVelBiasError.text

    bag = rosbag.Bag(bagFile, 'r')

    currentDistance = 0
    currentX = None
    currentY = None

    averageCurrent = 0
    count = 0

    for topic, msg, t in bag.read_messages(topics=['/v1/data_broadcaster/data']):
        if topic == '/v1/data_broadcaster/data':
            if currentX != None and currentY != None:
                currentDistance += math.sqrt(pow(currentX - msg.x, 2) + pow(currentY - msg.y,2))
            currentX = msg.x
            currentY = msg.y

            averageCurrent = (averageCurrent * count + math.sqrt(msg.u*msg.u + msg.v*msg.v)) / (count + 1)
            count += 1
            if math.sqrt(math.pow(currentX - beaconX,2) + math.pow(currentY - beaconY,2)) <= 1000:
                testDataEntry["success"] = True
                break

        if t.to_sec() > 2764800:
            testDataEntry["success"] = False
            break

    bag.close()

    testDataEntry["distance_travelled"] = currentDistance
    testDataEntry["average_current"] = averageCurrent
    testDataEntry["launch_file"] = launchFile

    print(testDataEntry, launchFile)
    return testDataEntry



def main(argv):
    testData = []
    for d in argv[1:-1]:
        print(d)
        for dirpath, dirnames, files in os.walk(d):
            statFile = None
            bagFile = None
            launchFile = None
            for name in files:
                if name.lower().endswith(".launch"):
                    launchFile = os.path.join(dirpath, name)
                if name == "stats.txt":
                    statFile = os.path.join(dirpath, name)
                if name.lower().endswith(".bag"):
                    bagFile = os.path.join(dirpath, name)
            if statFile != None and bagFile != None and launchFile != None:
                print(launchFile)
                testData.append(genTestDataEntry(launchFile, statFile, bagFile))
    
    f = open(argv[-1],"wb")
    pickle.dump(testData,f)
    f.close()

if __name__ == "__main__":
    main(sys.argv)