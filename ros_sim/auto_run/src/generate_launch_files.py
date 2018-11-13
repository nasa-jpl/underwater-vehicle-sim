import sys
import os
import random

import matplotlib.pyplot as plt

def replaceStartLocation(string, x, y, z):
    string = string.replace("<START_X>", str(x))
    string = string.replace("<START_Y>", str(y))
    string = string.replace("<START_Z>", str(z))
    return string

def main(argv):

    if len(argv) != 5:
        print("Invalid Arguments: Expected <inputFile> <outputDirectory> <spacing> <maxDistanceFromCenter>")
        sys.exit()

    inputFile = argv[1]
    outputDirectory = argv[2]
    spacing = float(argv[3])
    maxDistanceFromCenter = float(argv[4])
    randomizedFactor = 1500

    with open(inputFile, 'r') as file:
        exampleFile = file.read()


    plotX = []
    plotY = []

    files = []
    dimX = 0
    dimY = 0
    while dimX <= maxDistanceFromCenter:
        dimY = 0
        while dimY <= maxDistanceFromCenter:
            randomX = random.uniform(-randomizedFactor, randomizedFactor)
            randomY = random.uniform(-randomizedFactor, randomizedFactor)
            files.append(replaceStartLocation(exampleFile, dimX + randomX, dimY + randomY, 0))
            plotX.append(dimX + randomX)
            plotY.append(dimY + randomY)
            
            if dimY != 0:
                randomX = random.uniform(-randomizedFactor, randomizedFactor)
                randomY = random.uniform(-randomizedFactor, randomizedFactor)
                files.append(replaceStartLocation(exampleFile, dimX + randomX, -dimY + randomY, 0))
                plotX.append(dimX + randomX)
                plotY.append(-dimY + randomY)

            if dimX != 0:
                randomX = random.uniform(-randomizedFactor, randomizedFactor)
                randomY = random.uniform(-randomizedFactor, randomizedFactor)
                files.append(replaceStartLocation(exampleFile, -dimX + randomX, dimY + randomY, 0))
                plotX.append(-dimX + randomX)
                plotY.append(dimY + randomY)
                
                if dimY != 0:
                    randomX = random.uniform(-randomizedFactor, randomizedFactor)
                    randomY = random.uniform(-randomizedFactor, randomizedFactor)
                    files.append(replaceStartLocation(exampleFile, -dimX + randomX, -dimY + randomY, 0))
                    plotX.append(-dimX + randomX)
                    plotY.append(-dimY + randomY)

            
            dimY += spacing
        dimX += spacing

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    for i, file in enumerate(files):
        with open(os.path.join(outputDirectory, "planner_" + str(i) + ".launch"), "w+") as f:
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