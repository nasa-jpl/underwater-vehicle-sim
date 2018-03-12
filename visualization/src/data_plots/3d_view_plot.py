import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors

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

    data[measurement] = [max(d, 0.5) for d in data[measurement]]

    points = ax.scatter(data["x"][::dataRate], data["y"][::dataRate], data["h"][::dataRate], c=data[measurement][::dataRate], linewidth=0, cmap="plasma", norm=matplotlib.colors.LogNorm(vmin=min(data[measurement][::dataRate]), vmax=max(data[measurement][::dataRate])))

    xLim = ax.get_xlim()
    yLim = ax.get_ylim()

    ax.set_xlim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    ax.set_ylim([min(xLim[0], yLim[0]), max(xLim[1], yLim[1])])
    ax.set_zlim(ax.get_zlim()[0], 0)

    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_zlabel('Height (m)', fontsize=14)

    plt.scatter([0],[0],zs=[-1553], c='c', s=50, marker="^", label="Vent Source")
    plt.title("Hydrothermal Vent Search Simulation", fontsize=14)
    cbar = plt.colorbar(points)
    cbar.set_label("Neutrally Buoyant Tracer", fontsize=14)

    plt.show()

if __name__ == "__main__":
    main(sys.argv)
