import rosbag
import sys
import matplotlib.pyplot as plt
import math
import numpy as np

# phase should be "SPIRAL", "LAWNMOWER", "DYNAMIC_LAWNMOWER", "OTHER", or "UNSET" (if unimplemented)
def main(bagName, phase = "None"):
	bag = rosbag.Bag(bagName)
	x = []
	time = []
	y = []
	dye = []
	inPhase = False
	takeAll = False

	if phase == "None":
		takeAll = True

	for topic, msg, t in bag.read_messages(topics=['/v1/data_broadcaster/data', '/v1/plannerStatus']):
		if topic == '/v1/plannerStatus':
			if not inPhase and msg.data == phase:
				inPhase = True
			elif inPhase and msg.data != phase:
				inPhase = False
		else:
			if takeAll or inPhase:
				x.append(msg.y)
				y.append(msg.x)
				dye.append(msg.dye)
				time.append(msg.time.to_sec())

	bag.close()

#	plt.scatter(x, y, linewidth=0)
	plt.scatter(x, y, c=time, linewidth=0, cmap="plasma")
#	plt.scatter(x, y, c=np.log(dye), linewidth=0, cmap="plasma")
	plt.show()

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
			main(sys.argv[1], sys.argv[2])
	else:
		main(sys.argv[1])
