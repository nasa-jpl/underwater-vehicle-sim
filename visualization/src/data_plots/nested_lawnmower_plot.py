import sys
import matplotlib.pyplot as plt
import matplotlib.colors
import file_util
import operator
import bisect
import math
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

def plotData(data, data_inner, measurement):
    
    if data is None:
        print("Invalid filename")
        sys.exit()

    axes = plt.gca()

    axes.set_xlabel('X (m)', fontsize=14)
    axes.set_ylabel('Y (m)', fontsize=14)
    plt.title("Nested Lawnmower Survey", fontsize=18)
    
    plt.gcf().set_size_inches(14, 12)

    xExtent = [min(data["x"]), max(data["x"])]
    yExtent = [min(data["y"]), max(data["y"])]


    binCenter = [4381.08789,-7335.32910]
    binSize = 4000

    xBin = (binCenter[0] - xExtent[0]) / binSize
    if xBin < 0:
        xBin = int(math.floor(xBin))
    else:
        xBin = int(math.ceil(xBin))

    yBin = (binCenter[1] - yExtent[0]) / binSize
    if yBin < 0:
        yBin = int(math.floor(yBin))
    else:
        yBin = int(math.ceil(yBin))

    binOrigin = [binCenter[0] - (xBin * binSize), binCenter[1] - (yBin * binSize)]
    numBins = [int(math.ceil((xExtent[1] - binOrigin[0]) / binSize)),
               int(math.ceil((yExtent[1] - binOrigin[1]) / binSize))]

    binAverages = []
    binMax = []
    binCount = []
    for x in xrange(numBins[0]):
        binAverages.append([])
        binMax.append([])
        binCount.append([])
        for y in xrange(numBins[1]):
            binAverages[-1].append(0)
            binMax[-1].append(0)
            binCount[-1].append(0)


    for x,y,m in zip(data["x"], data["y"], data[measurement]):
        xBin = int(math.floor((x - binOrigin[0]) / binSize))
        yBin = int(math.floor((y - binOrigin[1]) / binSize))
        if not math.isnan(m):
            binAverages[xBin][yBin] += m
            binCount[xBin][yBin] += 1
            if binMax[xBin][yBin] < m:
               binMax[xBin][yBin] = m 

    for x in xrange(len(binAverages)):
        for y in xrange(len(binAverages[x])):
            if binCount[x][y] > 0:
                binAverages[x][y] /= binCount[x][y]


    axes.set_axis_bgcolor((0.8, 0.8, 0.8))

    maxima = []
    nonMaxima = []

    for xi in xrange(0, numBins[0]):
        for yi in xrange(0, numBins[1]):
            xOrigin = binOrigin[0] + (xi * binSize)
            yOrigin = binOrigin[1] + (yi * binSize)

            avg = binMax[xi][yi]

            if (xi > 0 and
               yi > 0 and
               xi < numBins[0] - 1 and
               yi < numBins[1] - 1 and
               avg > binMax[xi - 1][yi -1] and
               avg > binMax[xi - 1][yi] and
               avg > binMax[xi - 1][yi + 1] and
               avg > binMax[xi][yi - 1] and
               avg > binMax[xi][yi + 1] and
               avg > binMax[xi + 1][yi - 1] and
               avg > binMax[xi + 1][yi] and
               avg > binMax[xi + 1][yi + 1] and
               binCount[xi - 1][yi -1] > 0 and
               binCount[xi - 1][yi] > 0and
               binCount[xi - 1][yi + 1]> 0 and
               binCount[xi][yi - 1]> 0 and
               binCount[xi][yi + 1]> 0 and
               binCount[xi + 1][yi - 1]> 0 and
               binCount[xi + 1][yi]> 0 and
               binCount[xi + 1][yi + 1]> 0):
                maxima.append(Rectangle((xOrigin, yOrigin), 4000, 4000))
            else:
                nonMaxima.append(Rectangle((xOrigin, yOrigin), 4000, 4000))
                


    pcBelow = PatchCollection(maxima, facecolor='g', alpha=0.5,
                         edgecolor=None)
    
    pcAbove = PatchCollection(nonMaxima, facecolor=(0.8,0.8,0.8), alpha=1,
                         edgecolor=None)

    axes.add_collection(pcBelow)
    axes.add_collection(pcAbove)



    x = binOrigin[0]
    y = binOrigin[1]


    #5,3
    for x in xrange(9):
        plt.plot([binOrigin[0] + 4 * binSize + x * binSize / 3.0, binOrigin[0] + 4 * binSize + x * binSize / 3.0],
                 [binOrigin[1] + 2 * binSize, binOrigin[1] + 5 * binSize], c=(0.2,0.2,0.2), linestyle="--")

    first = True
    for y in xrange(9):
        if first:
            plt.plot([binOrigin[0] + 4 * binSize, binOrigin[0] + 7 * binSize],
                 [binOrigin[1] + 2 * binSize + y * binSize / 3.0, binOrigin[1] + 2 * binSize + y * binSize / 3.0],
                 c=(0.2,0.2,0.2), linestyle="--", label="Nested Bins")
            first = False
        else:
            plt.plot([binOrigin[0] + 4 * binSize, binOrigin[0] + 7 * binSize],
                 [binOrigin[1] + 2 * binSize + y * binSize / 3.0, binOrigin[1] + 2 * binSize + y * binSize / 3.0],
                 c=(0.2,0.2,0.2), linestyle="--")

    for x in xrange(numBins[0] + 1):
        plt.plot([binOrigin[0] + x * binSize, binOrigin[0] + x * binSize],[binOrigin[1], binOrigin[1] + numBins[1] * binSize],c='k', linewidth=3)

    for y in xrange(numBins[1] + 1):
        plt.plot([binOrigin[0], binOrigin[0] + numBins[0] * binSize], [binOrigin[1] + y * binSize, binOrigin[1] + y * binSize],c='k', linewidth=3)


    data[measurement] = [max(d, 0.1) for d in data[measurement]]
    points = plt.scatter(data["x"], data["y"], c=data[measurement], linewidth=0, cmap="plasma", norm=matplotlib.colors.LogNorm(vmin=min(data[measurement]), vmax=max(data[measurement])))
    
    plt.scatter(data_inner["x"], data_inner["y"], c='darkred', linewidth=0)
    plt.plot([],[],label="Planned Nested Survey", c='darkred', linewidth=3)

    cbar = plt.colorbar(points)
    cbar.set_label("Neutrally Buoyant Tracer")
    plt.scatter(data["x"][80], data["y"][80], c='k', marker="*", s=1000, label="Start Location")
    axes.set_xlim((binOrigin[0], binOrigin[0] + binSize * numBins[0]))
    axes.set_ylim((binOrigin[1], binOrigin[1] + binSize * numBins[1]))
    plt.legend(scatterpoints = 1, loc='lower right')
    #plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.png', dpi=100)
    #plt.gcf().savefig('/home/branch/Desktop/vent_search_map_plot.eps')

    plt.show()

def main(argv):
    if len(argv) != 6:
        print("Invalid Arguments")
        sys.exit()

    filename = argv[1]
    start_time = float(argv[2])
    end_time = float(argv[3])

    start_time_nested = float(argv[4])
    end_time_nested = float(argv[5])

    data = file_util.load_csv(filename)


    low = bisect.bisect_left(data['time'], start_time)
    hi = bisect.bisect_right(data['time'], end_time)

    low_nested = bisect.bisect_left(data['time'], start_time_nested)
    hi_nested = bisect.bisect_right(data['time'], end_time_nested)

    data_outer = {}
    data_inner = {}
    for k in data:
        data_outer[k] = data[k][low:hi]

    for k in data:
        data_inner[k] = data[k][low_nested:hi_nested]


    plotData(data_outer, data_inner, 'dye')
    

if __name__ == "__main__":
    main(sys.argv)