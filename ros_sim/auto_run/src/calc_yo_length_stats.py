import rosbag
import sys, math, os
import argparse
import numpy as np

from tqdm import tqdm

# calc score from bag file
# score is x,y distance from vent source
def main(bagNames, ventX = 0, ventY = 0):
	for bagName in bagNames:
		print("Reading " + bagName)
		bag = rosbag.Bag(bagName)
		yoLengths = []
		lastX = 0
		lastY = 0
		startX = 0
		startY = 0
		startedYo = False

		for topic, msg, t in tqdm(bag.read_messages(topics=['/v1/data_broadcaster/data', '/v1/plannerStatus',
												'model/model_time_offset', 'model/model_x_offset', 'model/model_y_offset'])):
			if topic == '/v1/plannerStatus':
				if msg.data == "YO TURN":
					if not startedYo:
						startedYo = True
						startX = lastX
						startY = lastY
					else:
						startedYo = False

						dist = math.sqrt((startX - lastX)**2 + (startY - lastY)**2)
						yoLengths.append(dist)
			elif topic == '/v1/data_broadcaster/data':
				lastX = msg.x
				lastY = msg.y
			else:
				print(msg)

		bag.close()

		yoLengths = np.array(yoLengths)

		if len(yoLengths) != 0:

			# calculate descriptive statistics
			meanYo = yoLengths.mean()
			minYo = yoLengths.min()
			maxYo = yoLengths.max()
			stdYo = yoLengths.std()
			medianYo = np.median(yoLengths)


			print("Yo Length Statistics:")
			print("mean = " + str(meanYo) + "m")
			print("std dev = " + str(stdYo) + "m")
			print("median = " + str(medianYo) + "m")
			print("n = " + str(len(yoLengths)))
			print("max = " + str(maxYo) + "m")
			print("min = " + str(minYo) + "m")
		else:
			print("Didn't yo")


if __name__ == "__main__":
	parser = argparse.ArgumentParser()

	parser.add_argument("bagName", help="the path of the bag file")
	parser.add_argument("-x", "--ventX", help="give x loc of vent source", type=float, default=0)
	parser.add_argument("-y", "--ventY", help="give y loc of vent source", type=float, default=0)

	args = parser.parse_args()

	if os.path.isfile(args.bagName):
		main([args.bagName], args.ventX, args.ventY)
	else:
		bagFiles = []

		for (dirpath, _, filenames) in os.walk(args.bagName):
			bagFiles.extend([os.path.join(dirpath, f) for f in filenames if f.endswith(".bag")])

		main(bagFiles, args.ventX, args.ventY)
