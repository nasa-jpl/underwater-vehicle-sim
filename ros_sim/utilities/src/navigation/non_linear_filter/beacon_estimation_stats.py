import numpy as np
import matplotlib.pyplot as plt
import sys
import math
import statistics
import pickle
import argparse


def plot(data):
    plt.scatter(data["vehicle_y"], data["vehicle_x"])
    plt.scatter(data["beacon_world_y"], data["beacon_world_x"])

    for x,y,r in zip(data["range_vx"], data["range_vy"], data["range_r"]):
        theta = np.linspace(-np.pi, np.pi, 200)
        circle_x = (np.sin(theta) * r) + x
        circle_y = (np.cos(theta) * r) + y

    #    plt.plot(circle_y, circle_x, c='r')
   # plt.scatter(range(len(v1_yaw)), v1_yaw)
    plt.show()

def find_runs_with_params(runs, params):
    inds = []
    for i,r in enumerate(runs):
        valid = True
        for k,v in params.items():
            if r[0][k] != v:
                valid = False
        if valid:
            inds.append(i)
    
    return inds

def plot_distance_vs_accuracy(data):
    beacon_loc = (0,0)
    accuracy = []
    distance = []
    
    for x,y,vx,vy in zip(data['beacon_world_x'],data['beacon_world_y'], data["beacon_vx"], data["beacon_vy"]):
        accuracy.append(math.sqrt(math.pow(beacon_loc[0] - x,2) + math.pow(beacon_loc[1] - y,2)))
        distance.append(math.sqrt(math.pow(vx - x,2) + math.pow(vy - y,2)))

    plt.scatter(distance, accuracy)
    plt.show()

def scatter_plot_data(run, x_names, y_names):
    print(run[0])
    for xn, yn in zip(x_names, y_names):
        x = run[1][xn]
        y = run[1][yn]
        plt.scatter(x,y, s=2, label=yn)

    plt.legend()
    plt.show()

def plot_beacon_estimate_stats(runs, plot_lines, beacon_loc=(0,0)):
    
    for line in plot_lines:
        estimates = {}
        print("Plot New Line")
        name = line[0]
        x_axis_param = line[1]
        other_params = line[2]

        for r in runs:
            run_params = r[0]
            run_data = r[1]

            valid = True
            for p in other_params:
                if r[0][p] != other_params[p]:
                    valid = False

            if valid:
                print("Has params: " + str(run_params))
                if run_params[x_axis_param] not in estimates:
                    print("Create new list for x_axis_param: " + str(run_params[x_axis_param]))
                    estimates[run_params[x_axis_param]] = []
                print("Adding to x_axis_param: " + str(run_params[x_axis_param]))
                for x,y in zip(run_data['beacon_world_x'],run_data['beacon_world_y']):
                    estimates[run_params[x_axis_param]].append(math.sqrt(math.pow(beacon_loc[0] - x,2) + math.pow(beacon_loc[1] - y,2)))

        estimate_list = list(estimates)
        estimate_list.sort()

        plot_x = []
        plot_y = []
        plot_y_std_dev = []
        for e in estimate_list:
            mean = statistics.mean(estimates[e])
            std_dev = statistics.stdev(estimates[e])

            plot_x.append(e)
            plot_y.append(mean)
            plot_y_std_dev.append(std_dev)

        plt.plot(plot_x, plot_y, label=name)
        print("")
        
    plt.legend()
    plt.show()

def print_params(runs):
    all_params = {}
    for r in runs:
        params = r[0]
        for k,v in params.items():
            if k not in all_params:
                all_params[k] = set()
            all_params[k].add(v)

    for k,v in all_params.items():
        param_str = ""
        for item in v:
            param_str += str(item) + ", "
        print(k + ": " + param_str[:-2])

def main(args):
    runs = pickle.load( open( args.pickle_filename, "rb" ) )
    if args.parameters:
        print_params(runs)
    elif args.scatter:
        params = {}
        params["MODEL_U"] = 0
        params["MODEL_V"] = 0
        params["USE_DVL"] = False
        params["FILTER_NUM_RANGES"] = 5
        run_inds = find_runs_with_params(runs, params)
        scatter_plot_data(runs[run_inds[0]], ("prop_t",),("prop_dx",))
    else: 
        other_params0 = {}
        other_params0["MODEL_U"] = -0.3
        other_params0["MODEL_V"] = 0.3
        other_params0["USE_DVL"] = True

        other_params1 = {}
        other_params1["MODEL_U"] =  -0.3
        other_params1["MODEL_V"] = 0.3
        other_params1["USE_DVL"] = False
        
        other_params2 = {}
        other_params2["MODEL_U"] = 0
        other_params2["MODEL_V"] = 0
        other_params2["USE_DVL"] = True

        other_params3 = {}
        other_params3["MODEL_U"] = 0
        other_params3["MODEL_V"] = 0
        other_params3["USE_DVL"] = False

        plot_lines = []
        plot_lines.append(("0", "FILTER_NUM_RANGES", other_params0))
        plot_lines.append(("1", "FILTER_NUM_RANGES", other_params1))
        plot_lines.append(("2", "FILTER_NUM_RANGES", other_params2))
        plot_lines.append(("3", "FILTER_NUM_RANGES", other_params3))

        plot_beacon_estimate_stats(runs, plot_lines)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process Beacon Estimation Runs')
    parser.add_argument('pickle_filename', type=str, help='Pickled data from processing script')
    parser.add_argument("-p", "--parameters", action="store_true", help="Output possible parameters")
    parser.add_argument("-s", "--scatter", action="store_true", help="Plot scatter of data")

    args = parser.parse_args()

    main(args)