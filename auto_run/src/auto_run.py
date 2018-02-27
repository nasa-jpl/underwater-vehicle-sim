import roslaunch
import rospy
import os
import shutil
import sys

from std_msgs.msg import String
import data_server.srv


currentGoal = "running"
print("TOP: " + currentGoal)
dataFilePub = None

def callback(data):
    global currentGoal
    currentGoal = data.data

def getLaunchFiles(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]

def runLaunchFile(uuid, filename, outputDirectory):
    launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])

    rospy.loginfo("Starting launch file: %s", filename)
    launch.start()

    rate = rospy.Rate(0.1)
    while currentGoal == 'running' :
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

    rospy.loginfo("Goal reached saving data to: %s", outputDirectory)

    os.makedirs(outputDirectory)
    dataFileClient = rospy.ServiceProxy('/data_server/save', data_server.srv.SaveData)
    dataFileClient(os.path.join(os.path.abspath(outputDirectory), "data.csv"))

    with open(os.path.join(outputDirectory, "stats.txt"), 'w+') as f:
        f.write("Goal State: " + currentGoal)

    shutil.copy(filename, outputDirectory)

    rospy.loginfo("Stopping launch file: %s", filename)
    launch.shutdown()

def main(argv):

    if len(argv) != 3:
        print("Invalid Arguments")
        sys.exit()

    inputDirectory = argv[1]
    outputDirectory = argv[2]


    rospy.init_node('en_Mapping', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber('/planner/goal', String, callback)

    launchFiles = getLaunchFiles(inputDirectory)

    outputDirectories = [os.path.join(outputDirectory, os.path.basename(file).replace('.','_')) for file in launchFiles]

    for launchFile, output in zip(launchFiles, outputDirectories):
        currentGoal = 'running'
        runLaunchFile(uuid, file, output)



if __name__ == "__main__":
    main(sys.argv)