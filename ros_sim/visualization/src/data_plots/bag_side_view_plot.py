import rosbag
import sys
import matplotlib.pyplot as plt


def main(bagName):
	bag = rosbag.Bag(bagName)
	time = []
	dye = []
	z = []
	for topic, msg, t in bag.read_messages(topics=['/v1/data_broadcaster/data']):
		time.append(msg.time.to_sec())
		dye.append(msg.dye)
		z.append(msg.h)

	bag.close()

	plt.scatter(time, z, c=dye, linewidth=0, cmap="plasma")
	plt.gca().invert_yaxis()
	plt.show()

if __name__ == "__main__":
    main(sys.argv[1])
