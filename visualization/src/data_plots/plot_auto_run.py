import sys
import matplotlib.pyplot as plt
import matplotlib.colors
import file_util
import map_view_plot
import os
import xml.etree.ElementTree as ET
import math

def getRunFiles(directory):
    runs = []
    i = 0
    for root, subdirs, files in os.walk(directory):
        run = {}
        for f in files:
            if f.endswith(".launch"):
                run["launch_file"] = os.path.join(root, f)
        if "stats.txt" in files:
            run["stats_file"] = os.path.join(root, "stats.txt")
        if "data.csv" in files:
            run["data_file"] = os.path.join(root, "data.csv")
        if "log.txt" in files:
            run["log_file"] = os.path.join(root, "log.txt")

        if len(run) > 0:
            print("Loading Run " + str(i))
            processRun(run)
            runs.append(run)
            i += 1
    return runs


def readStats(filename):
    stats = {}
    with open(filename, "r") as f:
        for line in f:
            if line == "Goal State: success":
                stats["success"] = True
            elif line == "Goal State: failed":
                stats["success"] = False
    return stats

def readLog(filename):
    log = []
    with open(filename, "r") as f:
        for line in f:
            logEntry = {}
            colonSplit = line.split(":")
            logEntry["time"] = float(colonSplit[0])
            logEntry["entry"] = colonSplit[1].strip().split(",")
            log.append(logEntry)

    return log

def readLaunch(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    launch = {}
    for param in root.findall('rosparam'):
        if "start_x" in param.get("param"):
            launch['start_x'] = float(param.text)
        elif "start_y" in param.get("param"):
            launch['start_y'] = float(param.text)

    return launch

def processRun(run):
    run["stats"] = readStats(run["stats_file"])
    #run["data"] = file_util.load_csv_first_and_last(run["data_file"])
    run["data"] = file_util.load(run["data_file"])
    run["launch"] = readLaunch(run["launch_file"])
    run["log"] = readLog(run["log_file"])

    run["stats"]["time"] = run["data"]["time"][-1] - run["data"]["time"][0]

    run["stats"]["spiral_time"] = 0
    run["stats"]["dynamic_lawnmower_time"] = 0
    run["stats"]["nested_lawnmower_time"] = 0

    for i in xrange(1, len(run["log"])):
        prevTime = run["log"][i - 1]["time"]
        currTime = run["log"][i]["time"]

        if prevTime <= 58 * 24 * 60 * 60:
            if run["log"][i - 1]["entry"][1] == "Spiral":
                run["stats"]["spiral_time"] += currTime - prevTime
            elif run["log"][i - 1]["entry"][1] == "DynamicLawnmower":
                run["stats"]["dynamic_lawnmower_time"] += currTime - prevTime
            elif run["log"][i - 1]["entry"][1] == "NestedLawnmower":
                run["stats"]["nested_lawnmower_time"] += currTime - prevTime

    #get rid of data so it can be garbage collected
  #  run["data"] = []

def displayRun(run):
    print(run["launch_file"])
    print(run["stats"]["success"])
    print(run["stats"]["time"])
    print(math.sqrt(run["launch"]["start_x"]**2 + run["launch"]["start_y"]**2))
   # map_view_plot.plotData(run["data"], "dye")
    print("")

def getSuccessRate(runs):
    success = 0
    for run in runs:
        if run["stats"]["success"]:
            success += 1

    return float(success) / len(runs)

def plotTimeVsDistance(runs):
    print("Plot Time Vs Distance")
    totalTime = []
    spiralTime = []
    dynamicTime = []
    nestedTime = []

    dist = []
    x = []
    y = []

    distFailed = []
    totalTimeFailed = []
    spiralTimeFailed = []
    dynamicTimeFailed = []
    nestedTimeFailed = []

    for run in runs:
        if run["stats"]["success"]:
            dist.append(math.sqrt(run["launch"]["start_x"]**2 + run["launch"]["start_y"]**2))
            x.append(run["launch"]["start_x"])
            y.append(run["launch"]["start_y"])
            totalTime.append(run["stats"]["time"] / 60 / 60 / 24)
            spiralTime.append(run["stats"]["spiral_time"] / 60 / 60 / 24)
            dynamicTime.append(run["stats"]["dynamic_lawnmower_time"] / 60 / 60 / 24)
            nestedTime.append(run["stats"]["nested_lawnmower_time"] / 60 / 60 / 24)    

        else:
           distFailed.append(math.sqrt(run["launch"]["start_x"]**2 + run["launch"]["start_y"]**2))        
           totalTimeFailed.append(28.0)
           spiralTimeFailed.append(run["stats"]["spiral_time"] / 60 / 60 / 24)
           dynamicTimeFailed.append(run["stats"]["dynamic_lawnmower_time"] / 60 / 60 / 24)
           nestedTimeFailed.append(run["stats"]["nested_lawnmower_time"] / 60 / 60 / 24)  




    totalColor = [0,0,0]
    totalColorFailed = [245 / 255.0, 91 / 255.0, 89 / 255.0]


    spiralColor = [0,0,0]
    spiralColorFailed = [245 / 255.0, 91 / 255.0, 89 / 255.0]

    dynamicColor = [0,0,0]
    dynamicColorFailed = [245 / 255.0, 91 / 255.0, 89 / 255.0]

    nestedStartColor = [0,0,0]
    nestedStartColorFailed = [245 / 255.0, 91 / 255.0, 89 / 255.0]

 #   totalColor = [10 / 255.0, 151 / 255.0, 158 / 255.0]
  #  totalColorFailed = [16 / 255.0, 243 / 255.0, 255 / 255.0]


   # spiralColor = [0, 0, 0]
   # spiralColorFailed = [0.5, 0.5, 0.5]

   # dynamicColor = [173 / 255.0, 64 / 255.0, 63 / 255.0]
   # dynamicColorFailed = [245 / 255.0, 91 / 255.0, 89 / 255.0]

   # nestedStartColor = [13 / 255.0, 90 / 255.0, 171 / 255.0]
   # nestedStartColorFailed = [19 / 255.0, 130 / 255.0, 247 / 255.0]

    marker_size = 30
    fig = plt.figure(1)
    
    fig.suptitle("Search Time vs. Distance from Vent Source", fontsize=32)

 #   plt.scatter(x, y, c=totalTime, linewidth=0, cmap='plasma',s=50)
    ax1 = plt.subplot(221)
    plt.scatter(dist, totalTime, c=totalColor, linewidth=0, s=marker_size)
    plt.scatter(distFailed, totalTimeFailed, c=totalColorFailed, linewidth=0, s=marker_size)
    plt.ylabel("Time (Days)", fontsize=24)
    plt.xlabel("Distance from Vent Source (m)", fontsize=24)
    plt.title("(a) Total Search Time", fontsize=28)
    plt.tick_params(axis='both', which='major', labelsize=24)
    ax1.set_xlim([0, 50000])
    ax1.set_ylim([0, ax1.get_ylim()[1]])

    ax2 = plt.subplot(222)
    plt.ylabel("Time (Days)", fontsize=24)
    plt.xlabel("Distance from Vent Source (m)", fontsize=24)
    plt.scatter(dist, spiralTime, c=spiralColor, linewidth=0, s=marker_size)
    plt.scatter(distFailed, spiralTimeFailed, c=spiralColorFailed, linewidth=0, s=marker_size)
    plt.title("(b) Spiral Survey Time", fontsize=28)
    plt.tick_params(axis='both', which='major', labelsize=24)
    ax2.set_xlim([0, 50000])
    ax2.set_ylim([0, ax2.get_ylim()[1]])

    ax3 =plt.subplot(223)
    plt.ylabel("Time (Days)", fontsize=24)
    plt.xlabel("Distance from Vent Source (m)", fontsize=24)
    plt.scatter(dist, dynamicTime, c=dynamicColor, linewidth=0, s=marker_size)
    plt.scatter(distFailed, dynamicTimeFailed, c=dynamicColorFailed, linewidth=0, s=marker_size)
    plt.title("(c) Dynamic Lawnmower Survey Time", fontsize=28)
    plt.tick_params(axis='both', which='major', labelsize=24)
    ax3.set_xlim([0, 50000])
    ax3.set_ylim([0, ax3.get_ylim()[1]])

    ax4 = plt.subplot(224)
    plt.ylabel("Time (Days)", fontsize=24)
    plt.xlabel("Distance from Vent Source (m)", fontsize=24)
    pSuccess = plt.scatter(dist, nestedTime, c=nestedStartColor, linewidth=0, s=marker_size)
    pFailed = plt.scatter(distFailed, nestedTimeFailed, c=nestedStartColorFailed, linewidth=0, s=marker_size)
    plt.title("(d) Nested Lawnmower Survey Time", fontsize=28)
    plt.tick_params(axis='both', which='major', labelsize=24)
    ax4.set_xlim([0, 50000])
    ax4.set_ylim([0, ax4.get_ylim()[1]])


    plt.figlegend( [pSuccess, pFailed], ["Successful Runs", "Failed Runs"], loc = 'lower center', ncol=5, labelspacing=0. , fontsize=24)
    plt.show()

def main(argv):
    if len(argv) != 2:
        print("Invalid Arguments")
        sys.exit()

    directory = argv[1]

    print("Loading Runs...")
    runs = getRunFiles(directory) 
   
    for run in runs:
        processRun(run)

    #plotTimeVsDistance(runs)
    print("Success Rate: " + str(getSuccessRate(runs)))        

    for run in runs:
        if run["launch_file"] == "./paper_output/6000m_30000m/planner_10_launch/planner_10.launch":
            map_view_plot.plotSurveyType(run["data"], run["log"])
        displayRun(run)

if __name__ == "__main__":
    main(sys.argv)