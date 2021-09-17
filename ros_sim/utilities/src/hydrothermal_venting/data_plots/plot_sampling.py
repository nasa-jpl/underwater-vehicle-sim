import plotly.graph_objects as go
import rosbag
import argparse
import numpy as np
import pickle
import os
import numpy as np
import bisect
import parse_bag

import dash
import dash_core_components as dcc
import dash_html_components as html

def eval_nested_keys(dict, keys):
    """
    Get the value from nested dictionary based on keys

    Parameters
    ----------
    dict : Dictionary
        Dictionary to get the values from
    keys : str or Iterable of str
        Nested keys
    """

    if type(keys) is str:
        keys = [keys]

    current_val = dict
    for k in keys:
        current_val = current_val[k]
    return current_val


def get_data(loaded_data, inputs):
    """
    Plot data

    Parameters
    ----------
    loaded_data : dictionary
        Figure to plot data
    inputs : Iterable of str
        data keys to use when plotting
    """
    return eval_nested_keys(loaded_data, inputs)

def get_data_percentile(loaded_data, inputs):
    """
    Plot data based on percentile of total data collected for that run

    Parameters
    ----------
    loaded_data : dictionary
        Figure to plot data
    inputs : (Iterable of str,Iterable of str,Iterable of str)
        Data keys for plotted data, data keys for percentile data, data keys for percentile data times
    """

    phases, phase_times = process_behavior_state(loaded_data)
    start_time = phase_times[phases.index("RUN_YOYO_LAWNMOWER")]

    sample_values = eval_nested_keys(loaded_data, inputs[0])

    data_values = eval_nested_keys(loaded_data, inputs[1])
    data_times = eval_nested_keys(loaded_data, inputs[2])
    data_values = data_values[bisect.bisect(data_times,start_time):]

    data_values.sort()

    percentile_values = []
    for s in sample_values:
        ind = bisect.bisect(data_values, s)
        percentile_values.append(float(ind)/len(data_values)*100.0)

    return percentile_values

def get_processed_times(loaded_data, inputs):
    """
    Plot times corrected to end of dynamic lawnmowers

    Parameters
    ----------
    loaded_data : dictionary
        Figure to plot data
    inputs : Iterable of str
        data keys to use when plotting
    """

    data = eval_nested_keys(loaded_data, inputs)
    phases, phase_times = process_behavior_state(loaded_data)
    start_time = phase_times[phases.index("RUN_YOYO_LAWNMOWER")]
    return [(t-start_time)/3600/24 for t in data]


def process_behavior_state(data):
    phases = []
    for p in eval_nested_keys(data, ["/v1/behavior_phase","strings"]):
        phases.append(p[0][2][0]) #[first string array][string array data][first value in string array]
    time = eval_nested_keys(data, ["/v1/behavior_phase","time"])

    current_phase = None

    compressed_phases = []
    compressed_times = []

    for p,t in zip(phases, time):
        if p != current_phase:
            compressed_phases.append(p)
            compressed_times.append(t)
        current_phase = p

    return compressed_phases,compressed_times

def plot_xy_per_method(fig, bag_file_directory, filenames, x_data_func, y_data_func, data_mapping, x_data_func_input=[], y_data_func_input=[], x_axis_label="", y_axis_label="", x_range=None, y_range=None):
    """
    Plot xy data with sample method as different colors

    Parameters
    ----------
    fig : plotly Figure
        Figure to plot data
    bag_file_directory : str
        Directory containing the files to load
    filenames : Dictionary
        Filenames of the files to load
    x_data_func : Function
        Function to call on the list of x data values to plot
    y_data_func : Function
        Function to call on the list of y data values to plot
    data_mapping : Dictionary of (str, str)
        Map methods to labels and colors. Each tuple represents (label, color)
    x_data_func_input : Object
        extra input to x_data_func
    y_data_func_input : Object
        extra input to y_data_func
    x_axis_label : str
        Label to use for the y axis
    y_axis_label : str
        Label to use for the y axis
    x_range : (float, float)
        Range for the x axis of the plot
    y_range : (float, float)
        Range for the y axis of the plot
    """

    x_data = {}
    y_data = {}
    color_data = {}

    for k,v in data_mapping.items():
        if k in filenames.keys():
            for f in filenames[k]:
                filename = os.path.join(bag_file_directory, f)
                print("Loading " + filename)
                loaded_data = pickle.load(open(filename,"rb"))
                if v[0] not in x_data:
                    x_data[v[0]] = []
                    y_data[v[0]] = []

                data = x_data_func(loaded_data, x_data_func_input)
                x_data[v[0]].extend(data)
                data = y_data_func(loaded_data, y_data_func_input)
                y_data[v[0]].extend(data)
                color_data[v[0]] = v[1]

    for k in x_data.keys():
        fig.add_trace(go.Scatter(x=x_data[k], y=y_data[k], mode='markers',name=k,
                                marker=dict(
                                    size=10,
                                    color=color_data[k],
                                    opacity=1.0
                                )))

    fig.update_xaxes(range=x_range)
    fig.update_yaxes(range=y_range)

    fig.update_layout(
        title="",
        xaxis_title=x_axis_label,
        yaxis_title=y_axis_label
    )

def plot_y_per_method(fig, bag_file_directory, filenames, data_func, data_mapping, data_func_input=[], y_axis_label="", category_order="mean ascending", range=None):
    """
    Plot some data value seperated by sample method

    Parameters
    ----------
    fig : plotly Figure
        Figure to plot data
    bag_file_directory : str
        Directory containing the files to load
    filenames : Dictionary
        Filenames of the files to load
    data_func : Function
        Function to call on the list of data values to plot
    data_mapping : Dictionary
        Map methods to labels
    data_func_input : Object
        extra input to data_func
    y_axis_label : str
        Label to use for the y axis
    category_order : str
        Sets the order of the categories on the x axis
    range : (float, float)
        Range for the y axis of the plot
    """

    data_points = []
    labeled_points = []

    for k,v in data_mapping.items():
        if k in filenames.keys():
            for f in filenames[k]:
                filename = os.path.join(bag_file_directory, f)
                print("Loading " + filename)
                loaded_data = pickle.load(open(filename,"rb"))
                data = data_func(loaded_data, data_func_input)
                data_points.extend(data)
                labeled_points.extend([v] * len(data))


    fig.add_trace(go.Scatter(x=labeled_points, y=data_points, mode='markers',
                            marker=dict(
                                size=10,
                                color='red',
                                opacity=1.0
                            )))
    fig.update_xaxes(type='category', categoryorder=category_order)
    fig.update_yaxes(range=range)

    fig.update_layout(
        title="",
        xaxis_title="Sample Selection Method",
        yaxis_title=y_axis_label
    )


def main():
    bag_file_directory = "/media/psf/Home/projs/ocean_worlds/ros_workspace/src/ros-underwater-sim/launch_files/nested_bin_site_selection/eval_runs/bag_files"
    filenames = {}
    filenames["revisit_on_complete"] = ["revisit_on_complete_1.p", "revisit_on_complete_2.p"]
    filenames["revisit_on_maxima"] = ["revisit_on_maxima_1.p", "revisit_on_maxima_2.p"]
    filenames["recursive_secretary_lt"] = ["recursive_secretary_3.p", "recursive_secretary_4.p"]
    filenames["recursive_secretary_mt"] = ["recursive_secretary_1.p", "recursive_secretary_2.p"]
    filenames["recursive_secretary_ht"] = ["recursive_secretary_5.p", "recursive_secretary_6.p"]

    fig = go.Figure()

    data_mapping = {}

    data_mapping["revisit_on_complete"] = "Revisit On Complete"
    data_mapping["revisit_on_maxima"] = "Revisit On Maxima"
    data_mapping["recursive_secretary_lt"] = "Recursive Secretary Low"
    data_mapping["recursive_secretary_mt"] = "Recursive Secretary Mid"
    data_mapping["recursive_secretary_ht"] = "Recursive Secretary High"

    #plot_y_per_method(fig, bag_file_directory, filenames, get_data, data_mapping, data_func_input=["/v1/sample/take_sample", "data"], y_axis_label="Measurment Value")
   # plot_y_per_method(fig, bag_file_directory, filenames, get_processed_times, data_mapping, data_func_input=["/v1/sample/take_sample", "time"], y_axis_label="Time (Days)")
    #plot_y_per_method(fig, bag_file_directory, filenames, get_data_percentile, data_mapping, data_func_input=(("/v1/sample/take_sample", "data"), 
    #                                                                                                        ("/v1/data_broadcaster/data", "dye"), 
    #                                                                                                        ("/v1/data_broadcaster/data", "time")), y_axis_label="Measurment Percentile", range=(-5,105))

    data_color_mapping = {}
    data_color_mapping["revisit_on_complete"] = ("Revisit On Complete", "red")
    data_color_mapping["revisit_on_maxima"] = ("Revisit On Maxima", "green")
    data_color_mapping["recursive_secretary_lt"] = ("Recursive Secretary Low", "blue")
    data_color_mapping["recursive_secretary_mt"] = ("Recursive Secretary Mid", "cyan")
    data_color_mapping["recursive_secretary_ht"] = ("Recursive Secretary High", "darkmagenta")

    plot_xy_per_method(fig, bag_file_directory, filenames, get_processed_times, get_data, data_color_mapping, x_data_func_input=["/v1/sample/take_sample", "time"], 
                                                                                                              y_data_func_input=["/v1/sample/take_sample", "data"],
                                                                                                              x_axis_label="Time",
                                                                                                              y_axis_label="Measurment Value")


    external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
    app = dash.Dash(external_stylesheets=external_stylesheets)

    app.layout = html.Div([
        dcc.Graph(id='g1', figure=fig, style={'height': '90vh', 'width': '90vh'})
    ])

    app.run_server(debug=False, host='0.0.0.0', port=8050)  # Turn off reloader if inside Jupyter

if __name__ == "__main__":
    main()