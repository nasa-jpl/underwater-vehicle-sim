import sys
import matplotlib.pyplot as plt

import file_util

def main(argv):
    if len(argv) != 3:
        print("Invalid Arguments")
        sys.exit()

    filename = argv[1]
    measurement = argv[2]

    data = file_util.load(filename)
    if data is None:
        print("Invalid filename")
        sys.exit()

    plt.scatter(data["x"], data["y"], c=data[measurement], linewidth=0, cmap="plasma")
    axes = plt.gca()

    xLim = axes.get_xlim()
    yLim = axes.get_ylim()

    axes.set_xlim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    axes.set_ylim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    
    plt.show()

if __name__ == "__main__":
    main(sys.argv)