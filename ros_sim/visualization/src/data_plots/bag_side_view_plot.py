import rosbag
import sys
import matplotlib.pyplot as plt

# phase should be "SPIRAL", "LAWNMOWER", "DYNAMIC_LAWNMOWER", "OTHER", or "UNSET" (if unimplemented)
def main(bagName, phase = "None"):
	bag = rosbag.Bag(bagName)
	time = []
	dye = []
	z = []
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
				time.append(msg.time.to_sec())
				dye.append(msg.dye)
				z.append(msg.h)

	bag.close()

	plt.scatter(time, z, c=dye, linewidth=0, cmap="plasma")
	plt.gca().invert_yaxis()
	plt.show()

if __name__ == "__main__":
	if len(sys.argv) > 2:
		phase = sys.argv[2]
		if phase != "SPIRAL" and phase != "LAWNMOWER" and phase != "DYNAMIC_LAWNMOWER" and phase != "OTHER" and phase != "UNSET":
			print(phase " is not a valid phase. Targets include: SPIRAL, LAWNMOWER, DYNAMIC_LAWNMOWER, OTHER, and UNSET")
			return 1
		main(sys.argv[1], sys.argv[2])
	else:
		main(sys.argv[1])
