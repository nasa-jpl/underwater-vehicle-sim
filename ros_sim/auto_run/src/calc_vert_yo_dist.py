import rosbag
import sys
import matplotlib.pyplot as plt
import matplotlib.colors
import pandas as pd
import numpy as np
import subprocess

from tqdm import tqdm


# phase should be "SPIRAL", "LAWNMOWER", "DYNAMIC_LAWNMOWER", "OTHER", or "UNSET" (if unimplemented)
def main(bagName, phase = "None", model = False):
	bag = rosbag.Bag(bagName)
	time = []
	dye = []
	z = []
	inPhase = False
	takeAll = False

	if phase == "None":
		takeAll = True

	for topic, msg, t in tqdm(bag.read_messages(topics=['/v1/data_broadcaster/data', '/v1/plannerStatus',
											'model/model_time_offset', 'model/model_x_offset', 'model/model_y_offset'])):
		if topic == '/v1/plannerStatus':
			if not inPhase and msg.data == phase:
				inPhase = True
			elif inPhase and msg.data != phase:
				inPhase = False
		elif topic == '/v1/data_broadcaster/data':
			if takeAll or inPhase:
				time.append(msg.time.to_sec())
				dye.append(msg.dye)
				z.append(msg.h)
		else:
			print(msg)

	bag.close()

	if model:
		# in python3, this is subprocess.run()
		subprocess.call(["./devel/lib/visualization/visualization_MODEL_FETCHER", bagName])
		df = pd.read_csv("out.csv")

		print("Read data, analyzing")

		maximum = df.loc[df.groupby('time').dye.idxmax(),:]
		maximum.reset_index(drop=True, inplace=True)

		sum = 0
		n = 0
		for i in tqdm(range(len(maximum))):
			if(maximum.loc[i]['dye'] >= 0.5):
				n += 1
				sum += abs(maximum.loc[i]['z'] - z[i*10])

		mean = sum/n
		print("Mean: " + str(mean))


if __name__ == "__main__":
	if len(sys.argv) > 2:
		phase = sys.argv[2]
		if (phase != "SPIRAL" and phase != "LAWNMOWER" and
			phase != "DYNAMIC_LAWNMOWER" and
			phase != "GRADIENT" and
			phase != "WAYPOINTS" and
			phase != "LINE0" and phase != "LINE1" and
			phase != "LINE2" and phase != "LINE3" and
			phase != "OTHER" and phase != "UNSET"):

			print(phase + " is not a valid phase. Targets include: SPIRAL, LAWNMOWER, DYNAMIC_LAWNMOWER, OTHER, and UNSET")
		else:
			main(sys.argv[1], sys.argv[2], model=True)
	else:
		main(sys.argv[1], model=True)
