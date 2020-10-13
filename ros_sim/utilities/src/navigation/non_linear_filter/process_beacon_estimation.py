import rosbag
import tf
import sys
import math
import os
import argparse
import pickle

def rotate_beacon_to_world(x,y,yaw,bx,by):
    rotated_bx = bx * math.cos(yaw) - by * math.sin(yaw)
    rotated_by = bx * math.sin(yaw) + by * math.cos(yaw)
    final_bx = x + rotated_bx
    final_by = y + rotated_by

    return final_bx, final_by

def load_params(params_filename):
    run_params = {}

    with open(params_filename) as file:
        for line in file:
            line_split = line.split()

            val = None
            try:
                val = float(line_split[1])
            except ValueError:
                pass
            
            if val is None:
                if line_split[1] == "true":
                    val = True
                elif line_split[1] == "false":
                    val = False

            run_params[line_split[0][1:-1]] = val
    return run_params

def load_bag(bag_filename):
    run_data = {}
    bag = rosbag.Bag(bag_filename, 'r')

    run_data["bag_filename"] = bag_filename
    run_data["v_x"] = []
    run_data["v_y"] = []
    run_data["v_dx"] = []
    run_data["v_dy"] = []
    run_data["v_yaw"] = []
    run_data["v_t"] = []

    run_data["dvl_dx"] = []
    run_data["dvl_dy"] = []
    run_data["dvl_t"] = []

    run_data["prop_dx"] = []
    run_data["prop_t"] = []

    run_data["range_r"] = []
    run_data["range_vx"] = []
    run_data["range_vy"] = []

    run_data["beacon_x"] = []
    run_data["beacon_y"] = []
    run_data["current_x"] = []
    run_data["current_y"] = []

    run_data["beacon_vx"] = []
    run_data["beacon_vy"] = []
    run_data["beacon_vyaw"] = []

    run_data["beacon_world_x"] = []
    run_data["beacon_world_y"] = []

    for topic, msg, t in bag.read_messages(topics=['/v1/true_nav/pose', '/v1/usbl/data', '/v1/single_beacon_nav/optimization_result', '/v1/dvl/data', '/v1/commanded_forward_velocity']):
        if topic == '/v1/true_nav/pose':
            run_data["v_t"].append(msg.header.stamp.to_sec())
            run_data["v_x"].append(msg.pose.pose.position.x)
            run_data["v_y"].append(msg.pose.pose.position.y)
            run_data["v_dx"].append(msg.twist.twist.linear.x)
            run_data["v_dy"].append(msg.twist.twist.linear.y)
            quaternion = (msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w)
            run_data["v_yaw"].append(tf.transformations.euler_from_quaternion(quaternion)[2])
        elif topic == '/v1/usbl/data':
            run_data["range_vx"].append(run_data["v_x"][-1])
            run_data["range_vy"].append(run_data["v_y"][-1])
            run_data["range_r"].append(msg.range)
        elif topic == '/v1/single_beacon_nav/optimization_result':
            run_data["beacon_vx"].append(run_data["v_x"][-1])
            run_data["beacon_vy"].append(run_data["v_y"][-1])
            run_data["beacon_vyaw"].append(run_data["v_yaw"][-1])
            run_data["beacon_x"].append(msg.data[0])
            run_data["beacon_y"].append(msg.data[1])
            run_data["current_x"].append(msg.data[2])
            run_data["current_y"].append(msg.data[3])

        elif topic == '/v1/dvl/data':
            run_data["dvl_dx"].append(msg.velocity.x)
            run_data["dvl_dy"].append(msg.velocity.y)
            run_data["dvl_t"].append(msg.header.stamp.to_sec())
        elif topic == '/v1/commanded_forward_velocity':
            run_data["prop_dx"].append(msg.data)
            run_data["prop_t"].append(msg.header.stamp.to_sec())        

    for x,y,yaw,bx,by in zip(run_data["beacon_vx"], run_data["beacon_vy"], run_data["beacon_vyaw"], run_data["beacon_x"], run_data["beacon_y"]):
        new_x, new_y = rotate_beacon_to_world(x,y,yaw,bx,by)
        run_data["beacon_world_x"].append(new_x)
        run_data["beacon_world_y"].append(new_y)

    bag.close()

    return run_data


def load_run(bag_filename, params_filename):
    run_params = load_params(params_filename)
    run_data = load_bag(bag_filename)

    return run_params, run_data

def load_all_runs(bag_directory, param_directory):
    matched_files = []
    runs = []

    bag_files = [f for f in os.listdir(bag_directory) if os.path.isfile(os.path.join(bag_directory, f))]
    param_files = [f for f in os.listdir(param_directory) if os.path.isfile(os.path.join(param_directory, f))]

    for bf in bag_files:
        for pf in param_files:
            bf_base = os.path.basename(bf)
            pf_base = os.path.basename(pf)
            if os.path.splitext(bf_base)[0] == os.path.splitext(pf_base)[0]:
                matched_files.append((os.path.join(bag_directory, bf), os.path.join(param_directory, pf)))

    for f in matched_files:
        print(f)
        runs.append(load_run(f[0], f[1]))

    return runs

def main(args):
    runs = load_all_runs(args.bag_directory, args.param_directory)
    pickle.dump(runs, open( args.output_file, "wb" ) )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process Beacon Estimation Runs')
    parser.add_argument('bag_directory', type=str, help='Directory with bag files')
    parser.add_argument('param_directory', type=str, help='Directory with param files')
    parser.add_argument('output_file', type=str, help='Output for pickle file')
    args = parser.parse_args()

    main(args)