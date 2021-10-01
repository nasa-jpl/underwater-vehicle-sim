import plotly.graph_objects as go
import rosbag
import argparse
import numpy as np
import pickle
import os
import numpy as np
import bisect
import parse_bag
import math

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
        if k not in current_val:
            return None
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
    inputs : (Iterable of str,Iterable of str,Iterable of str, bool)
        Data keys for plotted data, data keys for percentile data, data keys for percentile data times, take all values
    """
    if inputs[3]:
        start_time = 0
    else:
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

def update_list(list, func, args=[]):
    new_list = []
    for i in list:
        new_list.append(func(i, *args))

    return new_list
def seconds_to_days(time, start_time):
    return time/3600/24 - start_time/3600/24


def get_secretary_data(filename, submodular=False, filter_after_threshold=None):
    data = {}
    loaded_data = pickle.load(open(filename,"rb"))
    
    behavior_event_indicies = []

    for i,e in enumerate(loaded_data["/v1/behavior_event"]["strings"]):
        is_sampler_event = False
        for s in e:
            if s[0] == "type" and s[2][0] == "sampler_update":
                is_sampler_event = True
        if is_sampler_event:
            behavior_event_indicies.append(i)

    transitions = None
    thresholds = []
    threshold_times = []
    max_n_observations = []
    max_n_observation_times = []

    for i in behavior_event_indicies: #For each message
        for e in loaded_data["/v1/behavior_event"]["doubles"][i]: #For each double array in the message
            if e[0] == "transitions" and transitions is None:
                transitions = e[2]

    unfiltered_sample_values = eval_nested_keys(loaded_data, ("/v1/sample/take_sample", "data"))
    unfiltered_sample_times = eval_nested_keys(loaded_data, ("/v1/sample/take_sample", "time"))
    filtered_sample_values = []
    filtered_sample_times = []
    if filter_after_threshold is not None:
        for v,t in zip(unfiltered_sample_values, unfiltered_sample_times):
            include = True
            for trans_time in transitions:
                if t - trans_time > 0 and t - trans_time < filter_after_threshold:
                    include = False
                    break
            if include:
                filtered_sample_values.append(v)
                filtered_sample_times.append(t)
    else:
        filtered_sample_values = unfiltered_sample_values
        filtered_sample_times = unfiltered_sample_times

    data["sample_values"] = filtered_sample_values
    data["sample_time"] = filtered_sample_times


    data_step = 10

    data["sampler_start_time"] = loaded_data["/v1/behavior_event"]["time"][behavior_event_indicies[0]]
    data["sampler_end_time"] = loaded_data["/v1/behavior_event"]["time"][behavior_event_indicies[-1]]

    measurments = eval_nested_keys(loaded_data, ['/v1/data_broadcaster/data', 'dye'])[::data_step]
    measurment_times = eval_nested_keys(loaded_data, ['/v1/data_broadcaster/data', 'time'])[::data_step]
    start_index = bisect.bisect(measurment_times, data["sampler_start_time"])
    end_index = bisect.bisect(measurment_times, data["sampler_end_time"])
    data["filtered_measurments"] = measurments[start_index:]
    data["filtered_measurment_times"] = measurment_times[start_index:]


    #Get all threshold values
    temp_thresholds = []
    temp_threshold_times = []
    for i in behavior_event_indicies: #For each message
        for e in loaded_data["/v1/behavior_event"]["doubles"][i]: #For each double array in the message
            if e[0] == "current_threshold":
                temp_thresholds.append(e[2][0])
                temp_threshold_times.append(loaded_data["/v1/behavior_event"]["time"][i])
    temp_thresholds.append(temp_thresholds[-1])
    temp_threshold_times.append(data["filtered_measurment_times"][-1])

    #Update to merge same thresholds, expect include the final point
    for i,(v,t) in enumerate(zip(temp_thresholds, temp_threshold_times)):
        if (len(thresholds) == 0 or
            i == len(temp_thresholds) - 1 or
            abs(thresholds[-1] - v) > 0.0000001):
            thresholds.append(v)
            threshold_times.append(t)

    if submodular:
        thresholds[0] = -10
        for i,t in enumerate(threshold_times):
            for transition in transitions:
                if abs(t - transition) < 200:
                    thresholds[i] = -10
    else:
        thresholds[0] = -10

    for i in behavior_event_indicies: #For each message
        for e in loaded_data["/v1/behavior_event"]["doubles"][i]: #For each double array in the message
            if e[0] == "max_n_observations":
                max_n_observations.extend(e[2])
                max_n_observation_times.extend([loaded_data["/v1/behavior_event"]["time"][i]] * len(e[2]))

    data["transitions"] = transitions
    data["thresholds"] = thresholds
    data["threshold_times"] = threshold_times
    data["max_n_observations"] = max_n_observations
    data["max_n_observation_times"] = max_n_observation_times

    return data


def get_secretary_frame_data_at_time(data, time):
    frame_data = []
    end_index = bisect.bisect(data["filtered_measurment_times"], time)

    frame_data.append(go.Scatter(x=update_list(data["filtered_measurment_times"][:end_index], seconds_to_days, args=(data["sampler_start_time"],)), y=data["filtered_measurments"][:end_index], mode='lines',
                                line=dict(
                                color='grey',
                                ),
                                name="Measurment Data",
                                showlegend=True))

    end_index = bisect.bisect(data["sample_time"], time)

    frame_data.append(go.Scatter(x=update_list(data["sample_time"][:end_index], seconds_to_days, args=(data["sampler_start_time"],)), y=data["sample_values"][:end_index], mode='markers',
                                 marker=dict(
                                     size=10,
                                     color='red',
                                     opacity=1.0
                                 ),
                                 name="Samples",
                                 showlegend=True))

    end_index = bisect.bisect(data["threshold_times"], time)

    thresh_legend = True
    for i,(v, (t1,t2)) in enumerate(zip(data["thresholds"], zip(data["threshold_times"][:-1],data["threshold_times"][1:]))):
        if i < end_index:
            frame_data.append(go.Scatter(x=update_list([t1,t2], seconds_to_days, args=(data["sampler_start_time"],)), y=[v,v], mode='lines',
                                    line=dict(
                                    color='blue',
                                    ),
                                    name="Threshold",
                                    showlegend=thresh_legend))
            thresh_legend = False
        else:
            frame_data.append(go.Scatter(x=[], y=[], mode='lines',
                                    line=dict(
                                    color='blue',
                                    ),
                                    name="Threshold",
                                    showlegend=thresh_legend))
            thresh_legend = False

    return frame_data

def plot_animated_recursive_secretary_visualization(filename):
    data = get_secretary_data(filename)
    frames = []
    for t in range(int(data["sampler_start_time"]), int(data["sampler_start_time"] + (5*24*3600) + 1500), 1500):
            frames.append(go.Frame(data=get_secretary_frame_data_at_time(data, t)))

    fig = go.Figure(
        data=get_secretary_frame_data_at_time(data, data["sampler_start_time"]),
        layout=go.Layout(
            xaxis=dict(range=(update_list([data["sampler_start_time"] - 10], seconds_to_days, args=(data["sampler_start_time"],)) + [5]), autorange=False, zeroline=False),
            yaxis=dict(range=[-1, 50], autorange=False, zeroline=False),
            updatemenus=[dict(type="buttons",
                            buttons=[dict(label="Play",
                                            method="animate",
                                            args=[None, {"frame": {"duration": 75,"redraw": False},
                                                              "fromcurrent": True, 
                                                              "transition": {"duration": 0}}]
                                            )])]),
        frames=frames
    )

    for t in update_list(data["transitions"], seconds_to_days, args=(data["sampler_start_time"],)):
        fig.add_vline(x=t)

    fig.update_layout(
        xaxis_title="Time (Days)",
        yaxis_title="Measurment Value",
        font=dict(
            size=20,
        )
    )

    return fig


def plot_static_recursive_secretary_visualization(fig, filename):
    data = get_secretary_data(filename)
    
    for d in get_secretary_frame_data_at_time(data, data["sampler_end_time"] + 2000):
        fig.add_trace(d)

    print(data["transitions"], data["sampler_start_time"])
    for t in update_list(data["transitions"], seconds_to_days, args=(data["sampler_start_time"],)):
        fig.add_vline(x=t)

    fig.update_yaxes(range=[-1, 50])
    fig.update_xaxes(range=update_list([data["sampler_start_time"], data["sampler_end_time"]+2000], seconds_to_days, args=(data["sampler_start_time"],)))

    fig.update_layout(
        xaxis_title="Time (Days)",
        yaxis_title="Measurment Value",
        font=dict(
            size=20,
        )
    )

def plot_animated_submodular_secretary_visualization(filename):
    data = get_secretary_data(filename, submodular=True)
    frames = []
    for t in range(int(data["sampler_start_time"]), int(data["sampler_end_time"] + 5000), 500):
            frames.append(go.Frame(data=get_secretary_frame_data_at_time(data, t)))

    fig = go.Figure(
        data=get_secretary_frame_data_at_time(data, data["sampler_start_time"]),
        layout=go.Layout(
            xaxis=dict(range=update_list([data["sampler_start_time"] - 10, data["sampler_end_time"] + 2000], seconds_to_days, args=(data["sampler_start_time"],)), autorange=False, zeroline=False),
            yaxis=dict(range=[-1, 40], autorange=False, zeroline=False),
            updatemenus=[dict(type="buttons",
                            buttons=[dict(label="Play",
                                            method="animate",
                                            args=[None, {"frame": {"duration": 50,"redraw": False},
                                                              "fromcurrent": True, 
                                                              "transition": {"duration": 0}}]
                                            )])]),
        frames=frames
    )

    for t in update_list(data["transitions"], seconds_to_days, args=(data["sampler_start_time"],)):
        fig.add_vline(x=t)

    rect_starts = [data["sampler_start_time"]] + list(data["transitions"])
    rect_starts_corrected = update_list(rect_starts, seconds_to_days, args=(data["sampler_start_time"],))
    section_length = float(rect_starts_corrected[1]) - float(rect_starts_corrected[0])
    for t in rect_starts_corrected:
        rect_end = t + (section_length / math.e)
        fig.add_shape(type="rect",
            x0=t, y0=-10, x1=rect_end, y1=50,
            line=dict(
                color="rgba(147,112,219,0.0)",
                width=2,
            ),
            fillcolor="rgba(0,0,255,0.2)",
            )

    fig.update_layout(
        xaxis_title="Time (Days)",
        yaxis_title="Measurment Value",
        font=dict(
            size=20,
        )
    )
    return fig

def plot_static_submodular_secretary_visualization(fig, filename, filter_after_threshold=None):
    data = get_secretary_data(filename, submodular=True, filter_after_threshold=filter_after_threshold)
    for d in get_secretary_frame_data_at_time(data, data["sampler_end_time"] + 2000):
        fig.add_trace(d)

    for t in update_list(data["transitions"], seconds_to_days, args=(data["sampler_start_time"],)):
        fig.add_vline(x=t)

    
    rect_starts = [data["sampler_start_time"]] + list(data["transitions"])
    rect_starts_corrected = update_list(rect_starts, seconds_to_days, args=(data["sampler_start_time"],))
    section_length = float(rect_starts_corrected[1]) - float(rect_starts_corrected[0])
    for t in rect_starts_corrected:
        rect_end = t + (section_length / math.e)
        fig.add_shape(type="rect",
            x0=t, y0=-10, x1=rect_end, y1=50,
            line=dict(
                color="rgba(147,112,219,0.0)",
                width=2,
            ),
            fillcolor="rgba(0,0,255,0.1)",
            )

    fig.update_yaxes(range=[-1, 50])
    fig.update_xaxes(range=update_list([data["sampler_start_time"], data["sampler_end_time"]+2000], seconds_to_days, args=(data["sampler_start_time"],)))
    fig.update_layout(
        xaxis_title="Time (Days)",
        yaxis_title="Measurment Value",
        font=dict(
            size=20,
        )
    )

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

def plot_y_per_method(fig, bag_file_directory, filenames, data_func, data_mapping, data_func_input=[], y_axis_label="", category_order="mean ascending", color='indianred', range=None, box_plot=False, box_plot_order=None):
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
    color : Plotly Color
        Color of the plotted elements
    range : (float, float)
        Range for the y axis of the plot
    box_plot : bool
        Plot data as box plot instead of scatter plot
    box_plot_order : list or None
        order to plot data mapping values in
    """

    data_points = []
    labeled_points = []

    box_points = {}
    for k,v in data_mapping.items():
        if k in filenames.keys():
            for f in filenames[k]:
                filename = os.path.join(bag_file_directory, f)
                print("Loading " + filename)
                loaded_data = pickle.load(open(filename,"rb"))
                data = data_func(loaded_data, data_func_input)
                if data is not None:
                    data_points.extend(data)
                    labeled_points.extend([v] * len(data))
                    if v not in box_points.keys():
                        box_points[v] = []
                    box_points[v].extend(data)

    if not box_plot:
        fig.add_trace(go.Scatter(x=labeled_points, y=data_points, mode='markers',
                                marker=dict(
                                    size=10,
                                    color=color,
                                    opacity=1.0
                                )))
        fig.update_xaxes(type='category', categoryorder=category_order)

    else:
        if box_plot_order is not None:
            for k in box_plot_order:
                v = box_points[k]
                print(k, len(v))
                fig.add_trace(go.Box(y=v, boxpoints=False, jitter=0.3, pointpos=-1.8, name=k, fillcolor=color,line=dict(color='black'), showlegend=False,))
        else:
            for k,v in box_points.items():
                print(k, len(v))
                fig.add_trace(go.Box(y=v, boxpoints=False, jitter=0.3, pointpos=-1.8, name=k, fillcolor=color,line=dict(color='black'), showlegend=False))


    fig.update_yaxes(range=range)

    fig.update_layout(
        title="",
        xaxis_title="Sample Time Relative to Search",
        yaxis_title=y_axis_label,
        font=dict(
            size=20,
        )
    )

def main():
    bag_file_directory_small = "/media/psf/Home/projs/ocean_worlds/ros_workspace/src/ros-underwater-sim/launch_files/nested_bin_site_selection/eval_runs_small_sep/bag_files"
    bag_file_directory_large = "/media/psf/Home/projs/ocean_worlds/ros_workspace/src/ros-underwater-sim/launch_files/nested_bin_site_selection/eval_runs_large_sep/bag_files"

    bag_file_directory_fixed = "/media/psf/Home/projs/ocean_worlds/ros_workspace/src/ros-underwater-sim/launch_files/nested_bin_site_selection/eval_runs_fixed/bag_files"

    filenames = {}
    filenames["revisit_on_complete_maxima"] = ["revisit_on_complete_maxima_1.p", "revisit_on_complete_maxima_2.p"]
    filenames["revisit_on_complete_best"] = ["revisit_on_complete_best_1.p", "revisit_on_complete_best_2.p"]

    filenames["revisit_on_maxima"] = ["revisit_on_maxima_1.p", "revisit_on_maxima_2.p"]

    filenames["recursive_secretary_track_before_active_lt"] = ["recursive_secretary_track_before_active_3.p", "recursive_secretary_track_before_active_4.p"]
    filenames["recursive_secretary_track_before_active_mt"] = ["recursive_secretary_track_before_active_1.p", "recursive_secretary_track_before_active_2.p"]
    filenames["recursive_secretary_track_before_active_ht"] = ["recursive_secretary_track_before_active_5.p", "recursive_secretary_track_before_active_6.p"]
    filenames["recursive_secretary_track_before_active_mht"] = ["recursive_secretary_track_before_active_7.p", "recursive_secretary_track_before_active_8.p"]

    filenames["recursive_secretary_lt"] = ["recursive_secretary_3.p", "recursive_secretary_4.p"]
    filenames["recursive_secretary_mt"] = ["recursive_secretary_1.p", "recursive_secretary_2.p"]
    filenames["recursive_secretary_ht"] = ["recursive_secretary_5.p", "recursive_secretary_6.p"]
    filenames["recursive_secretary_mht"] = ["recursive_secretary_7.p", "recursive_secretary_8.p"]

    filenames["submodular_secretary_lt"] = ["submodular_secretary_3.p", "submodular_secretary_4.p"]
    filenames["submodular_secretary_mt"] = ["submodular_secretary_1.p", "submodular_secretary_2.p"]
    filenames["submodular_secretary_ht"] = ["submodular_secretary_5.p", "submodular_secretary_6.p"]
    filenames["submodular_secretary_mht"] = ["submodular_secretary_7.p", "submodular_secretary_8.p"]

    filenames["fixed_point_lt"] = ["fixed_point_3.p", "fixed_point_4.p"]
    filenames["fixed_point_mt"] = ["fixed_point_1.p", "fixed_point_2.p"]
    filenames["fixed_point_ht"] = ["fixed_point_5.p", "fixed_point_6.p"]
    filenames["fixed_point_mht"] = ["fixed_point_7.p", "fixed_point_8.p"]

    data_mapping = {}

 #   data_mapping["revisit_on_complete_maxima"] = "Revisit On Complete Maxima"
 #   data_mapping["revisit_on_complete_best"] = "Revisit On Complete"
 #   data_mapping["revisit_on_maxima"] = "Revisit On Maxima"

  #  data_mapping["recursive_secretary_track_before_active_lt"] = "Recursive Secretary (Track Before Active) Low"
  #  data_mapping["recursive_secretary_track_before_active_mt"] = "Recursive Secretary w/ Initial Survey"
  #  data_mapping["recursive_secretary_track_before_active_ht"] = "Recursive Secretary (Track Before Active) High"
  #  data_mapping["recursive_secretary_track_before_active_mht"] = "Recursive Secretary (Track Before Active) Mid High"

   # data_mapping["recursive_secretary_mt"] = "Recursive Secretary w/o Initial Survey"
    data_mapping["submodular_secretary_mt"] = "Submodular Secretary"

   # data_mapping["recursive_secretary_lt"] = "Less than"
   # data_mapping["recursive_secretary_mt"] = "Approx. Equal"
   # data_mapping["recursive_secretary_ht"] = "Greater Than"
    #data_mapping["recursive_secretary_mht"] = "Recursive Secretary Mid High"

  #  data_mapping["submodular_secretary_lt"] = "Less than"
  #  data_mapping["submodular_secretary_mt"] = "Approx. Equal"
  #  data_mapping["submodular_secretary_ht"] = "Greater Than"
 #   data_mapping["submodular_secretary_mht"] = "Submodular Secretary Mid High"

 #   data_mapping["fixed_point_lt"] = "Fixed Point Low"
    data_mapping["fixed_point_mt"] = "Fixed Interval"
 #   data_mapping["fixed_point_ht"] = "Fixed Point High"
 #   data_mapping["fixed_point_mht"] = "Fixed Point Mid High"


    fixed_path_filenames = {}

    fixed_path_filenames["recursive_secretary"] = ["recursive_secretary_1_start_1.p", "recursive_secretary_1_start_2.p","recursive_secretary_2_start_1.p", "recursive_secretary_2_start_2.p", "recursive_secretary_3_start_1.p", "recursive_secretary_3_start_2.p", "recursive_secretary_4_start_1.p", "recursive_secretary_4_start_2.p"]
    fixed_path_filenames["submodular_secretary"] = ["submodular_secretary_1_start_1.p", "submodular_secretary_1_start_2.p","submodular_secretary_2_start_1.p", "submodular_secretary_2_start_2.p", "submodular_secretary_3_start_1.p", "submodular_secretary_3_start_2.p", "submodular_secretary_4_start_1.p", "submodular_secretary_4_start_2.p"]
    fixed_path_filenames["fixed_point"] = ["fixed_point_1_start_1.p", "fixed_point_1_start_2.p","fixed_point_2_start_1.p", "fixed_point_2_start_2.p", "fixed_point_3_start_1.p", "fixed_point_3_start_2.p", "fixed_point_4_start_1.p", "fixed_point_4_start_2.p"]

    fixed_data_mapping = {}
    fixed_data_mapping["fixed_point"] = "Fixed Interval"
    fixed_data_mapping["recursive_secretary"] = "Recursive Secretary"
    fixed_data_mapping["submodular_secretary"] = "Submodular Secretary"


    fig = go.Figure()


    #plot_static_submodular_secretary_visualization(fig, os.path.join(bag_file_directory_large, filenames["submodular_secretary_mt"][0]), filter_after_threshold=None)
    #fig = plot_animated_submodular_secretary_visualization(os.path.join(bag_file_directory_large, filenames["submodular_secretary_lt"][1]))

    #plot_static_recursive_secretary_visualization(fig, os.path.join(bag_file_directory_small, filenames["recursive_secretary_mt"][0]))
    #fig = plot_animated_recursive_secretary_visualization(os.path.join(bag_file_directory_small, filenames["recursive_secretary_mt"][0]))
    
    #plot_y_per_method(fig, bag_file_directory_small, filenames, get_data, data_mapping, data_func_input=["/v1/sample/take_sample", "data"], y_axis_label="Measurment Value", box_plot=True, box_plot_order=["Fixed Interval", "Less than","Approx. Equal","Greater Than"])

   # plot_y_per_method(fig, bag_file_directory_fixed, fixed_path_filenames, get_data, fixed_data_mapping, data_func_input=["/v1/sample/take_sample", "data"], y_axis_label="Measurment Value", box_plot=True, box_plot_order=["Fixed Interval", "Submodular Secretary", "Recursive Secretary"])
    #plot_y_per_method(fig, bag_file_directory_small, filenames, get_data, data_mapping, data_func_input=["/v1/sample/take_sample", "data"], y_axis_label="Measurment Value", box_plot=True, box_plot_order=["Fixed Interval", "Revisit On Complete","Revisit On Maxima","Submodular Secretary","Recursive Secretary w/o Initial Survey","Recursive Secretary w/ Initial Survey"])
   # plot_y_per_method(fig, bag_file_directory, filenames, get_processed_times, data_mapping, data_func_input=["/v1/sample/take_sample", "time"], y_axis_label="Time (Days)")
    plot_y_per_method(fig, bag_file_directory_small, filenames, get_data_percentile, data_mapping, data_func_input=(("/v1/sample/take_sample", "data"), 
                                                                                                                    ("/v1/data_broadcaster/data", "dye"), 
                                                                                                                    ("/v1/data_broadcaster/data", "time"),
                                                                                                                    False), y_axis_label="Measurment Percentile", range=(-5,105), box_plot=True,box_plot_order=["Fixed Interval", "Revisit On Complete","Revisit On Maxima","Submodular Secretary","Recursive Secretary w/o Initial Survey","Recursive Secretary w/ Initial Survey"])

    #data_color_mapping = {}
    #data_color_mapping["revisit_on_complete"] = ("Revisit On Complete", "red")
    #data_color_mapping["revisit_on_maxima"] = ("Revisit On Maxima", "green")
    #data_color_mapping["recursive_secretary_lt"] = ("Recursive Secretary Low", "blue")
    #data_color_mapping["recursive_secretary_mt"] = ("Recursive Secretary Mid", "cyan")
    #data_color_mapping["recursive_secretary_ht"] = ("Recursive Secretary High", "darkmagenta")

    #plot_xy_per_method(fig, bag_file_directory, filenames, get_processed_times, get_data, data_color_mapping, x_data_func_input=["/v1/sample/take_sample", "time"], 
    #                                                                                                          y_data_func_input=["/v1/sample/take_sample", "data"],
    #                                                                                                          x_axis_label="Time",
    #                                                                                                          y_axis_label="Measurment Value")


    external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
    app = dash.Dash(external_stylesheets=external_stylesheets)

    app.layout = html.Div([
        dcc.Graph(id='g1', figure=fig, style={'height': '90vh', 'width': '90vh'})
    ])

    app.run_server(debug=False, host='0.0.0.0', port=8050)  # Turn off reloader if inside Jupyter

if __name__ == "__main__":
    main()