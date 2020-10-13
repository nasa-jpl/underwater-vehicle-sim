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

    plt.scatter(data["time"], data["h"], c=data[measurement], linewidth=0, cmap="plasma")
    plt.show()
    
if __name__ == "__main__":
    main(sys.argv)