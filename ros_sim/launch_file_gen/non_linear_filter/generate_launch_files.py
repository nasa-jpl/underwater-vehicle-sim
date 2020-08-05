import itertools
import numpy as np
import math
import os
import sys
import shutil

def outputFiles(launchTemplate, plannerTemplate, filterTemplate, outputDir, replaceMap):
    runNum = 0

    os.mkdir(outputDir)
    os.mkdir(os.path.join(outputDir, "filter_params"))
    os.mkdir(os.path.join(outputDir, "planner_params"))


    keys = [i[0] for i in replaceMap.items()]
    values = [i[1] for i in replaceMap.items()]

    for v in itertools.product(*values):
        launchOutFile = os.path.join(outputDir, "non_linear_filter_run_" + str(runNum) + ".launch")
        plannerOutFile = os.path.join(outputDir, "planner_params", "non_linear_filter_waypoints_" + str(runNum) + ".toml")
        filterOutFile = os.path.join(outputDir, "filter_params", "non_linear_filter_params_" + str(runNum) + ".toml")

        plannerOutFilename = os.path.join("planner_params", "non_linear_filter_waypoints_" + str(runNum) + ".toml")
        filterOutFilename = os.path.join("filter_params", "non_linear_filter_params_" + str(runNum) + ".toml")

        runNum += 1
        with open(launchTemplate, "rt") as fin:
            with open(launchOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        if type(k) is tuple:
                            for kk, vv in zip(k, v[i]):
                                line = line.replace(kk, str(vv))
                        elif type(k) is str:
                            line = line.replace(k, str(v[i]))
                    line = line.replace(":FILTER_FILENAME:", filterOutFilename)
                    line = line.replace(":PLANNER_FILENAME:", plannerOutFilename)
                    fout.write(line)
        
        with open(plannerTemplate, "rt") as fin:
            with open(plannerOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        if type(k) is tuple:
                            for kk, vv in zip(k, v[i]):
                                line = line.replace(kk, str(vv))
                        elif type(k) is str:
                            line = line.replace(k, str(v[i]))
                    fout.write(line)
        
        with open(filterTemplate, "rt") as fin:
            with open(filterOutFile, "wt") as fout:
                for line in fin:
                    for i, k in enumerate(keys):
                        if type(k) is tuple:
                            for kk, vv in zip(k, v[i]):
                                line = line.replace(kk, str(vv))
                        elif type(k) is str:
                            line = line.replace(k, str(v[i]))
                    fout.write(line)

if __name__ == "__main__":
    mbariReplaceMap = {}
    mbariReplaceMap[":RANGE_RANDOM_ERROR:"] = [0.05]
    mbariReplaceMap[":RANGE_BIAS_ERROR:"] = [0.05]

    mbariReplaceMap[":USBL_HERTZ:"] = [0.03333333333]
    mbariReplaceMap[":GYRO_RANDOM_ERROR:"] = [0]
    mbariReplaceMap[":GYRO_BIAS:"] = [0]
    mbariReplaceMap[":ANGULAR_VELOCITY_ACTIVE:"] = ["false"]

    mbariReplaceMap[":HEADING_RANDOM_ERROR:"] = [0.0349066]
    mbariReplaceMap[":HEADING_BIAS_ERROR:"] = [0]
    mbariReplaceMap[":HEADING_ACTIVE:"] = ["true"]

    mbariReplaceMap[":DVL_PERCENT_ERROR:"] = [0.02]
    mbariReplaceMap[":DVL_RANDOM_ERROR:"] = [0.002]

    mbariReplaceMap[":FILTER_NUM_RANGES:"] = [5,10,20,35,50]
    mbariReplaceMap[":USE_DVL:"] = ["true", "false"]

    mbariReplaceMap[(":MODEL_U:", ":MODEL_V:")] = [(0, 0), (-0.07071067811, 0.07071067811)]


    oceanWorldsReplaceMap = {}
    oceanWorldsReplaceMap[":RANGE_RANDOM_ERROR:"] = [0.05, 0.1]
    oceanWorldsReplaceMap[":RANGE_BIAS_ERROR:"] = [0.05, 0.1]

    oceanWorldsReplaceMap[":USBL_HERTZ:"] = [0.00833333333]
    oceanWorldsReplaceMap[":GYRO_RANDOM_ERROR:"] = [0.1, 0.4]
    oceanWorldsReplaceMap[":GYRO_BIAS:"] = [0.001, 0.01]
    oceanWorldsReplaceMap[":ANGULAR_VELOCITY_ACTIVE:"] = ["true"]

    oceanWorldsReplaceMap[":HEADING_RANDOM_ERROR:"] = [0]
    oceanWorldsReplaceMap[":HEADING_BIAS_ERROR:"] = [0]
    oceanWorldsReplaceMap[":HEADING_ACTIVE:"] = ["false"]

    oceanWorldsReplaceMap[":DVL_PERCENT_ERROR:"] = [0.02]
    oceanWorldsReplaceMap[":DVL_RANDOM_ERROR:"] = [0.002]

    oceanWorldsReplaceMap[":FILTER_NUM_RANGES:"] = [10,20,35]
    oceanWorldsReplaceMap[":USE_DVL:"] = ["true", "false"]

    oceanWorldsReplaceMap[(":MODEL_U:", ":MODEL_V:")] = [(0, 0), (-0.07071067811, 0.07071067811)]


    filter_template = "non_linear_filter_template.toml"
    launch_template = "non_linear_filter_template_run.launch"
    planner_template = "non_linear_filter_waypoints.toml"

    outputFiles(launch_template, planner_template, filter_template, sys.argv[1], oceanWorldsReplaceMap)