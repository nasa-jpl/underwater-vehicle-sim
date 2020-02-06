import rosbag
import sys, math
import argparse

from tqdm import tqdm

# calc score from bag file
# score is x,y distance from vent source
def main(bagName, ventX = 0, ventY = 0):
	bag = rosbag.Bag(bagName)
	maxDye = 0
	argmaxX = float('inf')
	argmaxY = float('inf')

	for topic, msg, t in tqdm(bag.read_messages(topics=['/v1/data_broadcaster/data'])):
		if topic == '/v1/data_broadcaster/data':
			if(msg.dye > maxDye):
				maxDye = msg.dye
				argmaxX = msg.x
				argmaxY = msg.y

	bag.close()

	score = math.sqrt((argmaxX - ventX)**2 + (argmaxY - ventY)**2)

	print("Dist from Source = " + str(score) + "m")

	return score


if __name__ == "__main__":
	parser = argparse.ArgumentParser()

	parser.add_argument("bagName", help="the path of the bag file")
	parser.add_argument("-x", "--ventX", help="give x loc of vent source", type=float, default=0)
	parser.add_argument("-y", "--ventY", help="give y loc of vent source", type=float, default=0)

	args = parser.parse_args()

	main(args.bagName, args.ventX, args.ventY)
