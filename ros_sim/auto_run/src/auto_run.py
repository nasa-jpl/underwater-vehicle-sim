import roslaunch
import rospy
import os
import shutil
import sys

from std_msgs.msg import String
import data_server.srv
import planner_log.srv


currentGoal = "running"
dataFilePub = None

def callback(data):
    global currentGoal
    currentGoal = data.data

def getLaunchFiles(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and f.endswith(".launch")]

def runLaunchFile(uuid, filename, outputDirectory, inputDirectory):
    launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])

    rospy.loginfo("Starting launch file: %s", filename)
    launch.start()

    rate = rospy.Rate(0.1)

    sawRunning = False
    while not sawRunning or currentGoal == 'running' :
        if(currentGoal == 'running'):
            sawRunning = True
            
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

    rospy.loginfo("Goal reached saving data to: %s", outputDirectory)

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    try:
        dataFileClient = rospy.ServiceProxy('/data_server/save', data_server.srv.SaveData)
        dataFileClient(os.path.join(os.path.abspath(outputDirectory), "data.csv"))
    except rospy.service.ServiceException:
        rospy.logerr("Auto Run: ServiceException /data_server/save: inputFile: %s", filename)
        launch.shutdown()
        return

    try:
        logClient = rospy.ServiceProxy('/planner_log/save', planner_log.srv.SaveLog)
        logClient(os.path.join(os.path.abspath(outputDirectory), "log.txt"))
    except rospy.service.ServiceException:
        rospy.logerr("Auto Run: ServiceException /planner_log/save: inputFile: %s", filename)
        launch.shutdown()
        return

    with open(os.path.join(outputDirectory, "stats.txt"), 'w+') as f:
        f.write("Goal State: " + currentGoal)

    shutil.copy(filename, outputDirectory)
    
    rospy.loginfo("Stopping launch file: %s", filename)
    launch.shutdown()

    shutil.move(filename, os.path.join(inputDirectory, "completed"))

def main(argv):
    global currentGoal

    if len(argv) != 3:
        print("Invalid Arguments")
        sys.exit()

    inputDirectory = argv[1]
    outputDirectory = argv[2]

    if not os.path.exists(os.path.join(inputDirectory, "completed")):
        os.makedirs(os.path.join(inputDirectory, "completed"))

    rospy.init_node('en_Mapping', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber('/planner/goal', String, callback)

    launchFiles = getLaunchFiles(inputDirectory)

    outputDirectories = [os.path.join(outputDirectory, os.path.basename(file).replace('.','_')) for file in launchFiles]

    for launchFile, output in zip(launchFiles, outputDirectories):
        currentGoal = 'running'
        runLaunchFile(uuid, launchFile, output, inputDirectory)



if __name__ == "__main__":
    main(sys.argv)