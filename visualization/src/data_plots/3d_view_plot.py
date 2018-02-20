import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import file_util

def main(argv):
    if len(argv) != 4:
        print("Invalid Arguments")
        sys.exit()

    filename = argv[1]
    measurement = argv[2]
    dataRate = int(argv[3])

    data = file_util.load(filename)
    if data is None:
        print("Invalid filename")
        sys.exit()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(data["x"][::dataRate], data["y"][::dataRate], data["h"][::dataRate], c=data[measurement][::dataRate], linewidth=0, cmap="plasma")

    xLim = ax.get_xlim()
    yLim = ax.get_ylim()

    ax.set_xlim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    ax.set_ylim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')

    plt.show()

if __name__ == "__main__":
    main(sys.argv)
