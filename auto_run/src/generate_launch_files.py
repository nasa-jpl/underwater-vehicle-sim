import sys
import os

def replaceStartLocation(string, x, y, z):
    string = string.replace("<START_X>", str(x))
    string = string.replace("<START_Y>", str(y))
    string = string.replace("<START_Z>", str(z))
    return string

def main(argv):

    if len(argv) != 5:
        print("Invalid Arguments: Expected inputFile outputDirectory spacing maxDistanceFromCenter")
        sys.exit()

    inputFile = argv[1]
    outputDirectory = argv[2]
    spacing = float(argv[3])
    maxDistanceFromCenter = float(argv[4])

    with open(inputFile, 'r') as file:
        exampleFile = file.read()

    files = []
    dimX = 0
    dimY = 0
    while dimX <= maxDistanceFromCenter:
        dimY = 0
        while dimY <= maxDistanceFromCenter:
            files.append(replaceStartLocation(exampleFile, dimX, dimY, 0))
            
            if dimY != 0:
                files.append(replaceStartLocation(exampleFile, dimX, -dimY, 0))

            if dimX != 0:
                files.append(replaceStartLocation(exampleFile, -dimX, dimY, 0))
                
                if dimY != 0:
                    files.append(replaceStartLocation(exampleFile, -dimX, -dimY, 0))

            
            dimY += spacing
        dimX += spacing

    if not os.path.exists(outputDirectory):
        os.makedirs(outputDirectory)

    for i, file in enumerate(files):
        with open(os.path.join(outputDirectory, "planner_" + str(i) + ".launch"), "w+") as f:
            f.write(file)


if __name__ == "__main__":
    main(sys.argv)