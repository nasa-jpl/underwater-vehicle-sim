import itertools
import numpy as np
import math
import os
import sys
import shutil

def outputFiles(launchTemplate, plannerTemplate, filterTemplate, outputDir, replaceMap):
    runNum = 0

    os.mkdir(outputDir)

    keys = [i[0] for i in replaceMap.items()]
    values = [i[1] for i in replaceMap.items()]

    for v in itertools.product(*values):
        launchOutFile = os.path.join(outputDir, "non_linear_filter_run_" + str(runNum) + ".launch")
        plannerOutFile = os.path.join(outputDir, "non_linear_filter_waypoints_" + str(runNum) + ".toml")
        filterOutFile = os.path.join(outputDir, "non_linear_filter_params_" + str(runNum) + ".toml")

        runNum += 1
        with open(launchTemplate, "rt") as fin:
            with open(launchOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        line = line.replace(k, v[i])
                    line = line.replace(":FILTER_FILENAME:", filterOutFile)
                    line = line.replace(":PLANNER_FILENAME:", plannerOutFile)
                    fout.write(line)
        
        with open(plannerTemplate, "rt") as fin:
            with open(plannerOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        line = line.replace(k, v[i])
                    line = line.replace()
                    fout.write(line)
        
        with open(filterTemplate, "rt") as fin:
            with open(filterOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        line = line.replace(k, v[i])
                    line = line.replace()
                    fout.write(line)

if __name__ == "__main__":
    mbariReplaceMap = {}
    mbariReplaceMap[":RANGE_RANDOM_ERROR:"] = [0.05]
    mbariReplaceMap[":RANGE_BIAS_ERROR:"] = [0.05]

    mbariReplaceMap[":USBL_HERTZ:"] = [0.03333333333]
    mbariReplaceMap[":GYRO_RANDOM_ERROR:"] = [0]
    mbariReplaceMap[":GYRO_BIAS:"] = [0]
    mbariReplaceMap[":ANGULAR_VELOCITY_ACTIVE:"] = ["false"]

    mbariReplaceMap[":HEADING_RANDOM_ERROR:"] = [0.0174533]
    mbariReplaceMap[":HEADING_BIAS_ERROR:"] = [0]
    mbariReplaceMap[":HEADING_ACTIVE:"] = ["true"]

    mbariReplaceMap[":DVL_PERCENT_ERROR:"] = [0.01]
    mbariReplaceMap[":DVL_RANDOM_ERROR:"] = [0.001]

    mbariReplaceMap[":FILTER_NUM_RANGES:"] = [5,10,20,35,50]

    outputFiles(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], mbariReplaceMap)