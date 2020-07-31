import numpy as np
import matplotlib.pyplot as plt
import rosbag
import sys
import tf
import math
import statistics

def rotate_beacon_to_world(x,y,yaw,bx,by):
    rotated_bx = bx * math.cos(yaw) - by * math.sin(yaw)
    rotated_by = bx * math.sin(yaw) + by * math.cos(yaw)
    final_bx = x + rotated_bx
    final_by = y + rotated_by

    return final_bx, final_by

def extract_data(bag_file):
    bag = rosbag.Bag(bag_file, 'r')
    data = {}
    data["vehicle_x"] = []
    data["vehicle_y"] = []
    data["vehicle_yaw"] = []


    data["range_r"] = []
    data["range_vx"] = []
    data["range_vy"] = []

    data["beacon_x"] = []
    data["beacon_y"] = []
    data["beacon_vx"] = []
    data["beacon_vy"] = []
    data["beacon_vyaw"] = []

    data["beacon_world_x"] = []
    data["beacon_world_y"] = []

    for topic, msg, t in bag.read_messages(topics=['/v1/true_nav/pose', '/v1/usbl/data', '/v1/single_beacon_nav/optimization_result']):
        if topic == '/v1/true_nav/pose':
            data["vehicle_x"].append(msg.pose.pose.position.x)
            data["vehicle_y"].append(msg.pose.pose.position.y)
            quaternion = (msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)
            data["vehicle_yaw"].append(tf.transformations.euler_from_quaternion(quaternion)[2])

        elif topic == '/v1/usbl/data':
            data["range_vx"].append(data["vehicle_x"][-1])
            data["range_vy"].append(data["vehicle_y"][-1])
            data["range_r"].append(msg.range)
        elif topic == '/v1/single_beacon_nav/optimization_result':
            data["beacon_vx"].append(data["vehicle_x"][-1])
            data["beacon_vy"].append(data["vehicle_y"][-1])
            data["beacon_vyaw"].append(data["vehicle_yaw"][-1])
            data["beacon_x"].append(msg.x)
            data["beacon_y"].append(msg.y)

    for x,y,yaw,bx,by in zip(data["beacon_vx"], data["beacon_vy"], data["beacon_vyaw"], data["beacon_x"], data["beacon_y"]):
        new_x, new_y = rotate_beacon_to_world(x,y,yaw,bx,by)
        data["beacon_world_x"].append(new_x)
        data["beacon_world_y"].append(new_y)


    bag.close()

    return data

def plot(data):
    plt.scatter(data["vehicle_y"], data["vehicle_x"])
    plt.scatter(data["beacon_world_y"], data["beacon_world_x"])

    for x,y,r in zip(data["range_vx"], data["range_vy"], data["range_r"]):
        theta = np.linspace(-np.pi, np.pi, 200)
        circle_x = (np.sin(theta) * r) + x
        circle_y = (np.cos(theta) * r) + y

        plt.plot(circle_y, circle_x, c='r')
   # plt.scatter(range(len(v1_yaw)), v1_yaw)
    plt.show()

def calc_stats(data):
    beacon_loc = (0,0)
    distances = []

    for x,y in zip(data['beacon_world_x'],data['beacon_world_y']):
        distances.append(math.sqrt(math.pow(beacon_loc[0] - x,2) + math.pow(beacon_loc[1] - y,2)))

    avg_dist = statistics.mean(distances)
    std_dev_dist = statistics.stdev(distances)
    return avg_dist, std_dev_dist


def main(argv):
    data = extract_data(argv[1])
    print(calc_stats(data))
    plot(data)

if __name__ == "__main__":
    main(sys.argv)