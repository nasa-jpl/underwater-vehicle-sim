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

	#for topic, msg, t in bag.read_messages(topics=['/v1/data_broadcaster/data', '/v1/plannerStatus']):
	for topic, msg, t in bag.read_messages(topics=['/v1/nav_filters/true_nav', '/v1/plannerStatus']):
		if topic == '/v1/plannerStatus':
			if not inPhase and msg.data == phase:
				inPhase = True
			elif inPhase and msg.data != phase:
				inPhase = False
		else:
			if takeAll or inPhase:
				x.append(msg.pose.pose.position.x)
				y.append(msg.pose.pose.position.y)
				#dye.append(msg.dye)
				time.append(t.to_sec())

	bag.close()


#	plt.scatter(x, y, linewidth=0)
	plt.scatter(y[::5], x[::5], linewidth=0, cmap="plasma", color='b',zorder=1000)
#	plt.scatter(x, y, c=np.log(dye), linewidth=0, cmap="plasma")

	beaconX = 10000
	beaconY = -10000
	heading = np.linspace(0, math.pi*2, 100)
	x = np.cos(heading) * 1000 + beaconX
	y = np.sin(heading) * 1000 + beaconY
	plt.plot(y, x, color='r')
	plt.scatter([beaconY], [beaconX], zorder=1, color='r')

	plt.gca().axis('scaled')
	plt.xlabel("X (Meters)")
	plt.ylabel("Y (Meters)")

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
