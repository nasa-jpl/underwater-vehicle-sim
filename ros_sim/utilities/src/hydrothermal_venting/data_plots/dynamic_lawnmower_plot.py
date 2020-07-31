import sys
import matplotlib.pyplot as plt
import matplotlib.colors
import file_util
import operator
import bisect
import math
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

def plotData(data, measurement):
    
    if data is None:
        print("Invalid filename")
        sys.exit()

    axes = plt.gca()

    axes.set_xlabel('X (m)', fontsize=28)
    axes.set_ylabel('Y (m)', fontsize=28)
    plt.title("Dynamic Lawnmower Survey", fontsize=32)
    
    plt.gcf().set_size_inches(14, 12)

   
    xLim = [min(data["x"]), max(data["x"])]
    yLim = [min(data["y"]) - 2000, max(data["y"]) + 2000]

    binAverages = []
    binMax = []
    binCount = []
    for x in xrange(int(math.ceil((xLim[1] - xLim[0]) / 4000.0))):
        binAverages.append([])
        binMax.append([])
        binCount.append([])
        for y in xrange(int(math.ceil((yLim[1] - yLim[0]) / 4000.0))):
            binAverages[-1].append(0)
            binMax[-1].append(0)
            binCount[-1].append(0)




    for x,y,m in zip(data["x"], data["y"], data[measurement]):
        xBin = int(math.floor((x - xLim[0]) / 4000.0))
        yBin = int(math.floor((y - yLim[0]) / 4000.0))
        if not math.isnan(m):
            binAverages[xBin][yBin] += m
            binCount[xBin][yBin] += 1
            if binMax[xBin][yBin] < m:
               binMax[xBin][yBin] = m 

    for x in xrange(len(binAverages)):
        for y in xrange(len(binAverages[x])):
            if binCount[x][y] > 0:
                preavg = binAverages[x][y]
                binAverages[x][y] /= binCount[x][y]


    axes.set_axis_bgcolor((0.8,0.8,0.8))

    aboveThresh = []
    belowThresh = []


    for xi, x in enumerate(binAverages):
        for yi, avg in enumerate(x):
            xOrigin = xLim[0] + (xi * 4000)
            yOrigin = yLim[0] + (yi * 4000)

            if xi < 5 and yi < 4:
                if math.isnan(avg) or avg < 0.5:
                    belowThresh.append(Rectangle((xOrigin, yOrigin), 4000, 4000))
                else:
                    aboveThresh.append(Rectangle((xOrigin, yOrigin), 4000, 4000))

            
                


    pcBelow = PatchCollection(aboveThresh, facecolor='g', alpha=0.5,
                         edgecolor=None)
    
    pcAbove = PatchCollection(belowThresh, facecolor=(0.8,0.8,0.8), alpha=1,
                         edgecolor=None)


    

    axes.add_collection(pcBelow)
    axes.add_collection(pcAbove)

    for xi, (avgX, maxX) in enumerate(zip(binAverages, binMax)):
        for yi, (avgY, maxY) in enumerate(zip(avgX, maxX)):
            xOrigin = xLim[0] + (xi * 4000)
            yOrigin = yLim[0] + (yi * 4000)
            if xi < 5 and yi < 4:
                if yi == 0 and (xi == 1 or xi == 2):
                    axes.text(xOrigin + 100, yOrigin + 3900, "Avg: " + ('%.2f' % avgY),
                        verticalalignment='top', horizontalalignment='left',
                        color='k', fontsize=24)
                else:
                    axes.text(xOrigin + 100, yOrigin + 3900, "Avg: " + ('%.2f' % avgY),
                        verticalalignment='top', horizontalalignment='left',
                        color='k', fontsize=24)
     

    
    
   # plt.legend()
    

    x = xLim[0]
    y = yLim[0]
    while x <= xLim[1]:
        if x + 4000 > xLim[1]:
            plt.plot([x + 60, x + 60],[yLim[0], yLim[1]],c='k', linewidth=5, label="Survey Boundary")
        else:
            plt.plot([x, x],[yLim[0], yLim[1]],c='k')

        x += 4000

    first = True
    while y <= yLim[1]:
        if first:
            plt.plot([xLim[0], xLim[1]], [y, y],c='k', linewidth=5)
            first = False
        else:
            plt.plot([xLim[0], xLim[1]], [y, y],c='k')
        y += 4000

    pointOver = {"x":[], "y":[], "m":[]}
    pointUnder = {"x":[], "y":[], "m":[]}

    for x,y,m in zip(data["x"], data["y"], data[measurement]):
        if m >= 0.5:
            pointOver["x"].append(x)
            pointOver["y"].append(y)
            pointOver["m"].append(m)
        else:
            pointUnder["x"].append(x)
            pointUnder["y"].append(y)
            pointUnder["m"].append(m)

    #plt.scatter(pointOver["x"], pointOver["y"], c='b', linewidth=0)
    #plt.scatter(pointUnder["x"], pointUnder["y"], c='m', linewidth=0)


    data[measurement] = [max(d, 0.1) for d in data[measurement]]
    points = plt.scatter(data["x"], data["y"], c=data[measurement], linewidth=0, cmap="plasma", norm=matplotlib.colors.LogNorm(vmin=min(data[measurement]), vmax=max(data[measurement])))
    cbar = plt.colorbar(points)
    cbar.set_label("Neutrally Buoyant Tracer", fontsize=28)
    cbar.ax.tick_params(labelsize=24) 
    plt.scatter(data["x"][80], data["y"][80], c='k', marker="*", s=1000, label="Survey Start Location")
    axes.set_xlim((xLim[0] - 50, xLim[1] + 50))
    axes.set_ylim((yLim[0] - 50, yLim[1] + 50))
    plt.legend(scatterpoints = 1, loc='lower right', fontsize=28)
    plt.tick_params(axis='both', which='major', labelsize=24)

    #plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.png', dpi=100)
    #plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.eps')

    plt.show()

def main(argv):
    if len(argv) != 4:
        print("Invalid Arguments")
        sys.exit()

    filename = argv[1]
    start_time = float(argv[2])
    end_time = float(argv[3])

    data = file_util.load_csv(filename)


    low = bisect.bisect_left(data['time'], start_time)
    hi = bisect.bisect_right(data['time'], end_time)

    for k in data:
        data[k] = data[k][low:hi]


    plotData(data, 'dye')
    

if __name__ == "__main__":
    main(sys.argv)