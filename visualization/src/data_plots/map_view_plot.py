import sys
import matplotlib.pyplot as plt
import matplotlib.colors
import file_util
import operator

def plot(filename, measurement):
    data = file_util.load(filename)
    plotData(data, measurement)

def plotData(data, measurement):
    
    if data is None:
        print("Invalid filename")
        sys.exit()

    data[measurement] = [max(d, 0.5) for d in data[measurement]]

    points = plt.scatter(data["x"], data["y"], c=data[measurement], linewidth=0, cmap="plasma", norm=matplotlib.colors.LogNorm(vmin=min(data[measurement]), vmax=max(data[measurement])))
    axes = plt.gca()

    xLim = axes.get_xlim()
    yLim = axes.get_ylim()

    
    
    axes.set_xlabel('X (m)')
    axes.set_ylabel('Y (m)')
    plt.title("Hydrothermal Vent Search Simulation")
    cbar = plt.colorbar(points)
    cbar.set_label("Neutrally Buoyant Tracer")
    plt.scatter([0],[0],c='k', marker="^", s=50, label="Vent Source")
    plt.legend()
    plt.axes().set_aspect('equal', 'datalim')
    plt.gcf().set_size_inches(14, 12)
    axes.set_xlim([-30000,30000])
    plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.png', dpi=100)
    plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.eps')
    #plt.show()

def plotSurveyType(data, log):
    spiralX = []
    spiralY = []

    dynamicX = []
    dynamicY = []

    nestedX = {}
    nestedY = {}

    spiralColor = [0, 0, 0]
    dynamicColor = [245 / 255.0, 91 / 255.0, 89 / 255.0]
    nestedStartColor = [16, 92, 207]
    nestedEndColor = [135, 201, 251]
    

    currentLog = 0
    for x,y,time in zip(data['x'], data['y'], data['time']):
        if currentLog < len(log) - 1 and time >= log[currentLog + 1]["time"]:
            currentLog += 1

        if log[currentLog]["entry"][1] == "Spiral":
            spiralX.append(x)
            spiralY.append(y)
        elif log[currentLog]["entry"][1] == "DynamicLawnmower":
            dynamicX.append(x)
            dynamicY.append(y)
        elif log[currentLog]["entry"][1] == "NestedLawnmower":
            if not int(float(log[currentLog]["entry"][5])) in nestedX:
                nestedX[int(float(log[currentLog]["entry"][5]))] = []
                nestedY[int(float(log[currentLog]["entry"][5]))] = []

            nestedX[int(float(log[currentLog]["entry"][5]))].append(x)
            nestedY[int(float(log[currentLog]["entry"][5]))].append(y)


    

    plt.scatter(spiralX, spiralY, c=spiralColor, linewidth=0, label="Spiral")
    plt.scatter(dynamicX, dynamicY, c=dynamicColor, linewidth=0, label="Dynamic Lawnmower")

    keyList = list(nestedX.keys())
    keyList.sort()
    keyList.reverse()

    colorStep = map(operator.sub, nestedEndColor, nestedStartColor)
    colorStep = [x / (len(keyList) - 1) for x in colorStep]

    for i, k in enumerate(keyList):
        thisStep = [x * i for x in colorStep]
        thisColor = [x / 255.0 for x in map(operator.add, nestedStartColor, thisStep)]

        label = str(k) + " m Nested Lawnmower"
        plt.scatter(nestedX[k], nestedY[k], c=thisColor, linewidth=0, label=label)

    plt.scatter([0],[0],c='k', marker="^", s=50, label="Vent Source")
    axes = plt.gca()

    

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Hydrothermal Vent Search Survey Type")
    plt.legend()
    plt.axes().set_aspect('equal', 'datalim')
    plt.gcf().set_size_inches(14, 12)
    axes.set_xlim([-30000,30000])
    plt.gcf().savefig('/home/branch/Desktop/vent_search_survey_type.png', dpi=100)
    plt.gcf().savefig('/home/branch/Desktop/vent_search_survey_type.eps')
  #  plt.show()



def main(argv):
    if len(argv) != 3:
        print("Invalid Arguments")
        sys.exit()

    filename = argv[1]
    measurement = argv[2]
    plot(filename, measurement)
    

if __name__ == "__main__":
    main(sys.argv)