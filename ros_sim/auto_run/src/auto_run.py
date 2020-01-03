import roslaunch
import rospy
import rosbag
import os
import shutil
import sys, time

import subprocess, shlex

from std_msgs.msg import String
import data_server.srv

import argparse

import psutil, signal


currentGoal = "running"
dataFilePub = None

def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

def reap_children(timeout=3):
    "Tries hard to terminate and ultimately kill all the children of this process."
    def on_terminate(proc):
        print("process {} terminated with exit code {}".format(proc, proc.returncode))

    procs = psutil.Process().children(recursive=True)

    if len(procs) == 0:
        print("No children to reap.")
    # send SIGTERM
    for p in procs:
        try:
            p.send_signal(signal.SIGINT)
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(procs, timeout=timeout, callback=on_terminate)
    if alive:
        # send SIGKILL
        for p in alive:
            print("process {} survived SIGINT; trying SIGTERM".format(p))
            try:
                p.terminate()
            except psutil.NoSuchProcess:
                pass
        gone, alive = psutil.wait_procs(procs, timeout=timeout, callback=on_terminate)
        if alive:
            # send SIGKILL
            for p in alive:
                print("process {} survived SIGTERM; trying SIGKILL".format(p))
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass
            gone, alive = psutil.wait_procs(alive, timeout=timeout, callback=on_terminate)
            if alive:
                # give up
                for p in alive:
                    print("process {} survived SIGKILL; giving up".format(p))

def callback(data):
    global currentGoal
    currentGoal = data.data

def getLaunchFiles(directory):
    return [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and f.endswith(".launch")]

def runLaunchFile(uuid, filename, outputDirectory, inputDirectory):
    print("UUID: " + str(uuid))
    launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    rospy.loginfo("Starting launch file: %s", filename)
    launch.start()

    rate = rospy.Rate(0.1)

    # start recording messages to bag
    # TODO TODO: this process won't die if you terminate the parent early (i.e. you ctr-c)
    command = "rosbag record -o " + outputDirectory + "/vehicleData /v1/data_broadcaster/data /v1/plannerStatus"
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)

    sawRunning = False
    while not sawRunning or currentGoal == 'running' :
        if(currentGoal == 'running'):
            sawRunning = True

        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

    print("Goal reached saving data\n")
    sys.stdout.flush()
    rospy.loginfo("Goal reached saving data to: %s", outputDirectory)

    # # this is crashing. It just saves that the run succeeded. Ignoring this for now
    # # additionally, it took awhile for the script to notice that we were done with the run, maybe something to look at?
    # #
    # try:
    #     dataFileClient = rospy.ServiceProxy('/data_server/save', data_server.srv.SaveData)
    #     dataFileClient(os.path.join(os.path.abspath(outputDirectory), "data.csv"))
    # except rospy.service.ServiceException:
    #     rospy.logerr("Auto Run: ServiceException /data_server/save: inputFile: %s", filename)
    #     launch.shutdown()
    #     # stop the bag recording
    #     rosbag_proc.send_signal(subprocess.signal.SIGINT)
    #     return

    # try:
    #     logClient = rospy.ServiceProxy('/planner_log/save', planner_log.srv.SaveLog)
    #     logClient(os.path.join(os.path.abspath(outputDirectory), "log.txt"))
    # except rospy.service.ServiceException:
    #     rospy.logerr("Auto Run: ServiceException /planner_log/save: inputFile: %s", filename)
    #     launch.shutdown()
    #     # stop the bag recording
    #     rosbag_proc.send_signal(subprocess.signal.SIGINT)
    #     return

    with open(os.path.join(outputDirectory, "stats.txt"), 'w+') as f:
        f.write("Goal State: " + currentGoal)

    # stop the bag recording
    print("Stopping bag file\n")
    rosbag_proc.send_signal(subprocess.signal.SIGINT)

    time.sleep(2.5)

    # # this was throwing an error for some reason (rosbaG_proc isn't a pid? doesnt make sense, it should work)
    # check if process was interrupted, if not escalate
    #if rosbag_proc.poll() is not None:
    #    print("Rosbag was not interrupted. Sending SIGKILL")
    #    rosbag_proc.send_signal(subprocess.signal.SIGKILL)

    shutil.copy(filename, outputDirectory)

    rospy.loginfo("Stopping launch file: %s", filename)
    launch.shutdown()

    time.sleep(10)

    print("Trying to stop bag file again")
    terminate_process_and_children(rosbag_proc)
    # check if we've shut down the launch
    while not rospy.is_shutdown():
        print("Launch Nodes didn't shut down properly. Retrying")
        launch.shutdown()

        print("Trying to reap children. Will need to relaunch roscore and maybe launchile UUID")
        reap_children()
        time.sleep(2.5)


    if os.path.exists(os.path.join(inputDirectory, "completed")):
        shutil.move(filename, os.path.join(inputDirectory, "completed"))

def main(input, outputDirectory):
    global currentGoal

    # check if input is file or directory
    # true if input is directory
    if not os.path.isfile(input):
        if not os.path.exists(os.path.join(input, "completed")):
            os.makedirs(os.path.join(input, "completed"))

        launchFiles = getLaunchFiles(input)

    roscore = subprocess.Popen('roscore')

    # if getting stuck here, need to run roscore in different process
    rospy.init_node('en_Mapping', anonymous=True)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    rospy.Subscriber('/v1/goal', String, callback)

    if not os.path.isfile(input):
        # run all launch files in dir
        outputDirectories = [os.path.join(outputDirectory, os.path.basename(file).replace('.','_')) for file in launchFiles]

        for launchFile, output in zip(launchFiles, outputDirectories):
            currentGoal = 'running'
            runLaunchFile(uuid, launchFile, output, input)
    else:
        # run the launch file
        currentGoal = 'running'
        runLaunchFile(uuid, input, outputDirectory, os.getcwd())



if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("input", help="path of launch file or directory containg launch files")
    parser.add_argument("outputDir", help="path of directory of output", nargs='*', default=None)

    args = parser.parse_args()

    if args.outputDir:
        outDir = args.outputDir[0]
    else:
        outDir = os.getcwd()

    main(args.input, outDir)
