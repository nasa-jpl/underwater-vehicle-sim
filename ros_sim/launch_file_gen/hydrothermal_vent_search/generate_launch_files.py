import sys
import os
import random

import matplotlib.pyplot as plt

def replaceStartLocation(string, x, y, z):
    string = string.replace("<START_X>", str(x))
    string = string.replace("<START_Y>", str(y))
    string = string.replace("<START_Z>", str(z))
    return string

def replaceConfigName(string, path):
    string = string.replace("<CONFIG_NAME>", str(path))
    return string

def main(argv):

    if len(argv) != 6:
        print("Invalid Arguments: Expected <inputFile> <tomlInputFile> <outputDirectory> <spacing> <maxDistanceFromCenter>")
        sys.exit()

    inputFile = argv[1]
    tomlInputFile = argv[2]
    outputDirectory = argv[3]
    spacing = float(argv[4])
    maxDistanceFromCenter = float(argv[5])
    randomizedFactor = 1500

    with open(inputFile, 'r') as file:
        exampleFile = file.read()
    with open(tomlInputFile, 'r') as file:
        exampleTomlFile = file.read()


    plotX = []
    plotY = []

    files = []
    configFiles = []
    dimX = 0
    dimY = 0
    while dimX <= maxDistanceFromCenter:
        dimY = 0
        while dimY <= maxDistanceFromCenter:
            randomX = random.uniform(-randomizedFactor, randomizedFactor)
            randomY = random.uniform(-randomizedFactor, randomizedFactor)
            startX = dimX + randomX
            startY = dimY + randomY
            files.append(replaceStartLocation(exampleFile, startX, startY, 0))
            configFiles.append(replaceStartLocation(exampleTomlFile, startX, startY, 0))
            plotX.append(startX)
            plotY.append(startY)

            if dimY != 0:
                randomX = random.uniform(-randomizedFactor, randomizedFactor)
                randomY = random.uniform(-randomizedFactor, randomizedFactor)
                startX = dimX + randomX
                startY = -dimY + randomY
                files.append(replaceStartLocation(exampleFile, startX, startY, 0))
                configFiles.append(replaceStartLocation(exampleTomlFile, startX, startY, 0))
                plotX.append(startX)
                plotY.append(startY)

            if dimX != 0:
                randomX = random.uniform(-randomizedFactor, randomizedFactor)
                randomY = random.uniform(-randomizedFactor, randomizedFactor)
                startX = -dimX + randomX
                startY = dimY + randomY
                files.append(replaceStartLocation(exampleFile, startX, startY, 0))
                configFiles.append(replaceStartLocation(exampleTomlFile, startX, startY, 0))
                plotX.append(startX)
                plotY.append(startY)

                if dimY != 0:
                    randomX = random.uniform(-randomizedFactor, randomizedFactor)
                    randomY = random.uniform(-randomizedFactor, randomizedFactor)
                    startX = -dimX + randomX
                    startY = -dimY + randomY
                    files.append(replaceStartLocation(exampleFile, startX, startY, 0))
                    configFiles.append(replaceStartLocation(exampleTomlFile, startX, startY, 0))
                    plotX.append(startX)
                    plotY.append(startY)


            dimY += spacing
        dimX += spacing

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    if not os.path.exists(os.path.join(outputDirectory, "planner_parameters")):
        os.makedirs(os.path.join(outputDirectory, "planner_parameters"))

    for i, file in enumerate(files):
        with open(os.path.join(outputDirectory, "planner_" + str(i) + ".launch"), "w+") as f:
            f.write(replaceConfigName(file, "config_" + str(i) + ".toml"))

    for i, file in enumerate(configFiles):
        with open(os.path.join(outputDirectory, "planner_parameters", "config_" + str(i) + ".toml"), "w+") as f:
            f.write(file)

    with open(os.path.join(outputDirectory, "info.txt"), 'w+') as f:
        f.write("spacing " + str(spacing) + "\n")
        f.write("maxDistanceFromCenter " + str(maxDistanceFromCenter) + "\n")
        f.write("center 0 0\n")
        f.write("outputDirectory " + outputDirectory  + "\n")
        f.write("inputFile " + inputFile  + "\n")

    plt.scatter(plotX, plotY)
    plt.show()
if __name__ == "__main__":
    main(sys.argv)
