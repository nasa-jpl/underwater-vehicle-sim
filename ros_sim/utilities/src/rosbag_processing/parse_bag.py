from genericpath import isdir
import rosbag
import argparse
import numpy as np
import pickle
import os


def parse_bag(bag_filename, topics):
    bag = rosbag.Bag(bag_filename)
    parsed_bag = {}
    for topic, msg, t in bag.read_messages(topics=topics):

        if msg._type == "underwater_vehicle_msgs/LogData":
            if topic not in parsed_bag:
                parsed_bag[topic] = {"strings":[], "ints":[], "doubles":[], "bytes":[], "time":[]}

            parsed_bag[topic]["strings"].append([])
            parsed_bag[topic]["ints"].append([])
            parsed_bag[topic]["doubles"].append([])
            parsed_bag[topic]["bytes"].append([])
            parsed_bag[topic]["time"].append(msg.header.stamp.to_sec())

            for m in msg.stringArrays:
                parsed_bag[topic]["strings"][-1].append((m.variable, m.units, m.data))

            for m in msg.intArrays:
                parsed_bag[topic]["ints"][-1].append((m.variable, m.units, m.data))

            for m in msg.doubleArrays:
                parsed_bag[topic]["doubles"][-1].append((m.variable, m.units, m.data))

            for m in msg.byteArrays:
                parsed_bag[topic]["bytes"][-1].append((m.variable, m.units, m.data))
        elif msg._type == "underwater_vehicle_msgs/TakeSample":
            if topic not in parsed_bag:
                parsed_bag[topic] = {"time":[], "location":[], "data":[]}
            parsed_bag[topic]["time"].append(msg.header.stamp.to_sec())
            parsed_bag[topic]["location"].append([msg.location.x, msg.location.y, msg.location.z])
            parsed_bag[topic]["data"].append(msg.data)

        elif msg._type == "underwater_vehicle_msgs/VehicleData":
            if topic not in parsed_bag:
                parsed_bag[topic] = {"time":[], "x":[], "y":[], "h":[], "sonar_depth":[], "temp":[], "salt":[], "dye":[], "u":[], "v":[]}

            parsed_bag[topic]["time"].append(msg.time.to_sec())
            parsed_bag[topic]["x"].append(msg.x)
            parsed_bag[topic]["y"].append(msg.y)
            parsed_bag[topic]["h"].append(msg.h)
            parsed_bag[topic]["sonar_depth"].append(msg.sonarDepth)
            parsed_bag[topic]["temp"].append(msg.temp)
            parsed_bag[topic]["salt"].append(msg.salt)
            parsed_bag[topic]["dye"].append(msg.dye)
            parsed_bag[topic]["u"].append(msg.u)
            parsed_bag[topic]["v"].append(msg.v)

    return parsed_bag

def generate_output_filename(input_file, input_dir, output_dir):
    base = os.path.basename(input_file)
    filename = os.path.splitext(base)[0] + ".p"
    relative_dir = os.path.relpath(os.path.dirname(input_file), input_dir)
    if output_dir is None:
        output_dir = input_dir
        
    return os.path.join(output_dir, relative_dir, filename)

def main(args):
    input_files = []
    output_files = []

    if os.path.isdir(args.rosbag):
        for subdir, _, files in os.walk(args.rosbag):
            for file in files:
                if file.endswith(".bag"):
                    input_files.append(os.path.join(subdir, file))
                    output_files.append(generate_output_filename(input_files[-1], args.rosbag, args.output))
    elif  os.path.isfile(args.rosbag):
        if args.rosbag.endswith(".bag"):
            input_files.append(args.rosbag)
            input_dir = os.path.dirname(args.rosbag)
            output_files.append(generate_output_filename(input_files[-1], input_dir, args.output))

    for i,o in zip(input_files, output_files):
        if args.overwrite or not os.path.exists(o):
            print("Processing " + i)
            o_dir = os.path.dirname(o)
            if not os.path.exists(o_dir):
                os.makedirs(o_dir)
            pickle.dump(parse_bag(i, args.topics), open(o, "wb"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert bag files to pickled data for faster loading.')
    parser.add_argument("rosbag", type=str, help="Filename or directory of the rosbag file(s) to parse")
    parser.add_argument("-o", "--output", type=str, help="Output directory for pickle files")
    parser.add_argument('-w', "--overwrite", action='store_true', help="Overwrite existing parsed files")
    parser.add_argument('-t', '--topics', nargs='+', default=None, help="ROS topics to parse")

    args = parser.parse_args()
    main(args)