import itertools
import numpy as np
import math
import os
import sys
import shutil

def outputFiles(launchTemplate, tomlTemplate, outputDir):
    locations = [(10000,10000, 100),
                (-10000,10000, 100),
                (10000,-10000, 100),
                (-10000,-10000, 100)]

    replaceMap = {}
    replaceMap[":RANGE_RANDOM_ERROR:"] = [0.05]
    replaceMap[":RANGE_BIAS_ERROR:"] = [0.05]

    replaceMap[":USBL_HERTZ:"] = [0.01666666666]
    replaceMap[":GYRO_RANDOM_ERROR:"] = [0.004363323]
    replaceMap[":GYRO_BIAS:"] = [0.00026179939]

    runNum = 0

    os.mkdir(outputDir)

    for l in locations:
        startLocations = []
        for i in np.linspace(0, 2*math.pi, 8, endpoint=False):
            startLocations.append((math.cos(i) * 10000 + l[0], math.sin(i) * 10000 + l[1], l[2]))

        keys = [i[0] for i in replaceMap.items()]
        values = [i[1] for i in replaceMap.items()]

        for v in itertools.product(*values):
            for sl in startLocations:
                outFile = os.path.join(outputDir, "golden_selection_homing_run_" + str(runNum) + ".launch")

                runNum += 1
                with open(launchTemplate, "rt") as fin:
                    with open(outFile, "wt") as fout:
                        for line in fin:
                            for i, k in enumerate(keys):
                                line = line.replace(k, "{:.9f}".format(v[i]))
                            line = line.replace(":START_X:", "{:.9f}".format(sl[0]))
                            line = line.replace(":START_Y:", "{:.9f}".format(sl[1]))
                            line = line.replace(":START_Z:", "{:.9f}".format(sl[2]))

                            line = line.replace(":BEACON_X:", "{:.9f}".format(l[0]))
                            line = line.replace(":BEACON_Y:", "{:.9f}".format(l[1]))

                            fout.write(line)
                shutil.copyfile(tomlTemplate, os.path.join(os.path.dirname(outFile), "golden_selection_homing.toml"))

if __name__ == "__main__":
    outputFiles(sys.argv[1], sys.argv[2], sys.argv[3])