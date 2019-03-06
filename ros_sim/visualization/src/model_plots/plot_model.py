from netCDF4 import Dataset

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
from numpy import linspace, meshgrid
from matplotlib.mlab import griddata
from matplotlib import colors
import os

import numpy as np
import sys

def loadFVCOM(filename, time):
    fvcom = Dataset(filename, "r")

    fvcomData = []

    # get the netCDF dimensions
    nodeDim = fvcom.dimensions["node"]
    timeDim = fvcom.dimensions["time"]
    siglayDim = fvcom.dimensions["siglay"]

    nodeToLoad = [True] * len(nodeDim)
    siglayToLoad = [True] * len(siglayDim)
    timeToLoad = [False] * len(timeDim)

    timeToLoad[time] = True

    #variable fields in netCDF file
    hVar = fvcom.variables["h"] #bathymetry at node
    xVar = fvcom.variables["x"] #x at node
    yVar = fvcom.variables["y"] #y at node
    dyeVar = fvcom.variables["DYE"] #dye at node
    siglayVar = fvcom.variables["siglay"]

    #We load in h variable data early because we need it to calculate dye and siglays
    loadedH = hVar[:]

    nodeDye = [[] for n in xrange(len(nodeDim))]
    nodeDepth = [[] for n in xrange(len(nodeDim))]

    for s in dyeVar[timeToLoad, siglayToLoad, nodeToLoad][0]:
        for ni, n in enumerate(s):
            nodeDye[ni].append(n)

    for s in siglayVar[siglayToLoad, nodeToLoad]:
        for ni, n in enumerate(s):
            nodeDepth[ni].append(-n * loadedH[ni])

    for x,y,z, h,dye in zip(xVar[:],
                       yVar[:],
                       nodeDepth,
                       loadedH,
                       nodeDye):
        fvcomData.append((x, y, z, h, dye))

    return fvcomData

def filterXY(fvcomData, xInterval, yInterval):
    fvcomData = [d for d in fvcomData if (xInterval[0] <= d[0] <= xInterval[1] and yInterval[0] <= d[1] <= yInterval[1])]
    return fvcomData

def dye_plot(fvcomData, plotType, siglay, minColor=-1, maxColor=-1):

    x = []
    y = []
    z = []
    depth = []
    dye = []

    for d in fvcomData:
        x.append(d[0])
        y.append(d[1])
        z.append(d[2][siglay])
        depth.append(d[3]),
        dye.append(d[4][siglay])

    if plotType == "3d":
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("Dye, siglay " + str(siglay))
        ax.scatter(x, y, z, c=dye)
        ax.invert_zaxis()
    elif plotType == "contour":
        fig = plt.figure()
        ax = fig.add_subplot(111, axisbg='black')
        ax.set_title("Dye, siglay " + str(siglay))
        resX = 10
        resY = 10
        xi = linspace(min(x), max(x), resX)
        yi = linspace(min(y), max(y), resY)
        Z = griddata(x, y, dye, xi, yi, interp="linear")
        X, Y = meshgrid(xi, yi)

        levels = None
        if minColor == 0:
            minColor = 0.0001
        elif minColor == -1:
            minColor = min(dye)

        if maxColor == -1:
            maxColor = max(dye)

        lev_exp = np.arange(np.floor(np.log10(minColor)-1),
                            np.ceil(np.log10(maxColor)+1))
        levs = np.power(10, lev_exp)
        cf = ax.contourf(X, Y, Z, levs, norm=colors.LogNorm(), cmap=plt.get_cmap('inferno'))

        fig.colorbar(cf, ax=ax)


def bathymetry_plot(fvcomData, plotType):

    x = []
    y = []
    z = []
    depth = []
    dye = []

    for d in fvcomData:
        x.append(d[0])
        y.append(d[1])
        z.append(d[2][0])
        depth.append(d[3]),
        dye.append(d[4][0])

    if plotType == "3d":
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("Bathymetry")
        ax.scatter(x, y, h)
        ax.invert_zaxis()
    elif plotType == "contour":
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("Bathymetry")
        resX = 10
        resY = 10
        xi = linspace(min(x), max(x), resX)
        yi = linspace(min(y), max(y), resY)
        Z = griddata(x, y, depth, xi, yi, interp="linear")
        X, Y = meshgrid(xi, yi)
        levels = MaxNLocator(nbins=15).tick_values(min(depth), max(depth))
        cf = ax.contourf(X, Y, Z, levels=levels, cmap=plt.get_cmap('YlGnBu'))
        fig.colorbar(cf, ax=ax)

def main():
    fvcomFile = sys.argv[1]
    plotType = sys.argv[2]
    outputDir = sys.argv[3]

    fvcomData = loadFVCOM(fvcomFile, 1)

    fvcomData = filterXY(fvcomData, (-50000, 50000), (-50000, 50000))

    for i in xrange(0,127,2):
        print("Output: " + str(i))
        dye_plot(fvcomData, plotType, i, minColor=0, maxColor=100)

        filename = format(i, '03') + ".png"
        plt.savefig(os.path.join(outputDir, filename), bbox_inches='tight')


if __name__== "__main__":
    main()
