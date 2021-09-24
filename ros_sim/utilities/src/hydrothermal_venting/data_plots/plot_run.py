from bisect import bisect

from numpy.lib.function_base import insert
import plotly.graph_objects as go
import rosbag
import argparse
import numpy as np
import pickle
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

def process_behavior_state(data, vehicle_namespace):
    phases = []
    for p in eval_nested_keys(data, ["/" + vehicle_namespace + "/behavior_phase","strings"]):
        phases.append(p[0][2][0]) #[first string array][string array data][first value in string array]
    time = eval_nested_keys(data, ["/" + vehicle_namespace + "/behavior_phase","time"])

    current_phase = None

    compressed_phases = []
    compressed_times = []

    for p,t in zip(phases, time):
        if p != current_phase:
            compressed_phases.append(p)
            compressed_times.append(t)
        current_phase = p

    return compressed_phases,compressed_times
 
def phase_at_time(beahvior_phases, behavior_phase_times, curr_time):
    insert_index = bisect.bisect(behavior_phase_times, curr_time)

    if insert_index == 0:
        return beahvior_phases[0]
    
    return beahvior_phases[insert_index-1]

def filter_by_phase(beahvior_phases, behavior_phase_times, data, data_times, phases):

    filtered_data = []
    filtered_data_times = []

    for p, st, et in zip(beahvior_phases, behavior_phase_times[:-1], behavior_phase_times[1:]):
        if p in phases:
            start_index = bisect.bisect(data_times, st)
            end_index = bisect.bisect(data_times, et)
            filtered_data.extend(data[start_index:end_index])
            filtered_data_times.extend(data_times[start_index:end_index])

    return filtered_data, filtered_data_times


def plot_histogram(fig_histogram, bag, vehicle_namespace, measurment_type="dye", phases=None):
    """
    Plot histogram of collected data

    Parameters
    ----------
    fig_histogram : plotly Figure
        Figure to plot data histogram
    bag : Dictionary
        rosbag file data to plot from
    vehicle_namespace : str
        The vehicle namespace for the data to plot
    measurment_type : str
        The measurment type to plot (dye,temp,salt)
    phases : None, str, or List of str
        Behavior phases to plot in the histogram
    """

    if type(phases) is str:
        phases = [phases]

    behavior_phase, behavior_phase_time = process_behavior_state(bag, vehicle_namespace)

    unfiltered_data = bag['/' + vehicle_namespace + '/data_broadcaster/data'][measurment_type]
    unfiltered_data_times = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"]
    if phases is None:
        filtered_data = unfiltered_data
    else:
        filtered_data,_ = filter_by_phase(behavior_phase, behavior_phase_time, unfiltered_data, unfiltered_data_times, phases)


    unfiltered_samples = bag['/' + vehicle_namespace + '/sample/take_sample']["data"]
    unfiltered_sample_times = bag['/' + vehicle_namespace + '/sample/take_sample']["time"]
    if phases is None:
        filtered_samples = bag['/' + vehicle_namespace + '/sample/take_sample']["data"]
    else:
        filtered_samples,_ = filter_by_phase(behavior_phase, behavior_phase_time, unfiltered_samples, unfiltered_sample_times, phases)


    fig_histogram.add_trace(go.Histogram(x=filtered_data))
    for s in filtered_samples:
        fig_histogram.add_vline(x=s)

def plot_data(bag, vehicle_namespace, measurment_type="dye", data_step=1, fig_3d=None, fig_2d=None, fig_depth=None, fig_data=None, log_data=True, plot_threshold=0, phases=None):
    """
    Plot data collected by a vehicle.

    Parameters
    ----------
    bag : Dictionary
        rosbag file to plot from
    vehicle_namespace : str
        The vehicle namespace for the data to plot
    measurment_type : str
        The measurment type to plot (dye,temp,salt)
    data_step : int
        Every N messages to plot. i.e. 1 is plot every message, 2 is plot every other message.
    fig_3d : plotly Figure
        Figure to plot in 3d
    fig_2d : plotly Figure
        Figure to plot in 2d
    fig_depth : plotly Figure
        Figure to plot depth over time
    fig_data : plotly Figure
        Figure to plot data over time
    log_data : bool
        Take the log of the data for color before plotting
    phases : None, str, or List of str
        Behavior phases to plot in the histogram
    """

    if type(phases) is str:
        phases = [phases]

    behavior_phase, behavior_phase_time = process_behavior_state(bag, vehicle_namespace)

    start_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][0]
    end_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][-1]

    x = bag['/' + vehicle_namespace + '/data_broadcaster/data']["y"][::data_step]
    y = bag['/' + vehicle_namespace + '/data_broadcaster/data']["x"][::data_step]
    z = [-h for h in bag['/' + vehicle_namespace + '/data_broadcaster/data']["h"][::data_step]]
    data = [max(plot_threshold,d) for d in bag['/' + vehicle_namespace + '/data_broadcaster/data'][measurment_type][::data_step]]
    
    data_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][::data_step]
    data_sonar = bag['/' + vehicle_namespace + '/data_broadcaster/data']["sonar_depth"][::data_step]

    sample_x = []
    sample_y = []
    sample_z = []
    for x_loc,y_loc,z_loc in bag['/' + vehicle_namespace + '/sample/take_sample']["location"]:
        sample_x.append(y_loc)
        sample_y.append(x_loc)
        sample_z.append(-z_loc)

    sample_data = bag['/' + vehicle_namespace + '/sample/take_sample']["data"]
    sample_time = bag['/' + vehicle_namespace + '/sample/take_sample']["time"]

    #Filter by phase if needed
    if phases is not None:
        x,_ = filter_by_phase(behavior_phase, behavior_phase_time, x, data_time, phases)
        y,_ = filter_by_phase(behavior_phase, behavior_phase_time, y, data_time, phases)
        z,_ = filter_by_phase(behavior_phase, behavior_phase_time, z, data_time, phases)
        data,_ = filter_by_phase(behavior_phase, behavior_phase_time, data, data_time, phases)
        data_sonar,_ = filter_by_phase(behavior_phase, behavior_phase_time, data_sonar, data_time, phases)
        data_time,_ = filter_by_phase(behavior_phase, behavior_phase_time, data_time, data_time, phases)

        sample_x,_ = filter_by_phase(behavior_phase, behavior_phase_time, sample_x, sample_time, phases)
        sample_y,_ = filter_by_phase(behavior_phase, behavior_phase_time, sample_y, sample_time, phases)
        sample_z,_ = filter_by_phase(behavior_phase, behavior_phase_time, sample_z, sample_time, phases)
        sample_data,_ = filter_by_phase(behavior_phase, behavior_phase_time, sample_data, sample_time, phases)
        sample_time,_ = filter_by_phase(behavior_phase, behavior_phase_time, sample_time, sample_time, phases)

    #Convert times to hours after filtering
    data_time = [(t - start_time)/3600 for t in data_time]
    sample_time = [(t - start_time)/3600 for t in sample_time]

    clean_data = data
    if log_data:
        data = np.log(data)


    if fig_3d is not None:
        fig_3d.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='markers',
            marker=dict(
                size=6,
                color=data,
                colorscale='Viridis',
                opacity=0.8
            )))

        fig_3d.add_trace(go.Scatter3d(x=sample_x, y=sample_y, z=sample_z, mode='markers',
            marker=dict(
                size=10,
                color='red',
                opacity=0.8
            )))

    if fig_2d is not None:
        fig_2d.add_trace(go.Scatter(x=x, y=y, mode='markers',
            marker=dict(
                    size=6,
                    color=data,
                    colorscale='Viridis',
                    opacity=0.8
            )))
        
        fig_2d.add_trace(go.Scatter(x=sample_x, y=sample_y, mode='markers',
            marker=dict(
                size=10,
                color='red',
                opacity=0.8
            )))

    if fig_depth is not None:
        fig_depth.add_trace(go.Scatter(x=data_time, y=z, mode='markers',
            marker=dict(
                    size=6,
                    color=data,
                    colorscale='Viridis',
                    opacity=0.8
            )))

        fig_depth.add_trace(go.Scatter(x=sample_time, y=sample_z, mode='markers',
            marker=dict(
                size=10,
                color='red',
                opacity=0.8
            )))

        bathy = [z_val - s_val for z_val, s_val in zip(z, data_sonar)]
        fig_depth.add_trace(go.Scatter(x=data_time, y=bathy, mode='lines'))

    if fig_data is not None:
        fig_data.add_trace(go.Scatter(x=data_time, y=clean_data, mode='lines'))

        fig_data.add_trace(go.Scatter(x=sample_time, y=sample_data, mode='markers',
            marker=dict(
                size=10,
                color='red',
                opacity=0.8
            )))

    return start_time, end_time
def main(args):
    print("Opening and Parsing ROS Bag")
    if args.filename.endswith(".bag"):
        file_data = parse_bag.parse_bag(args.filename, None)
    elif args.filename.endswith(".p"):
        file_data = pickle.load(open(args.filename,"rb"))
    print("Opened and Parsed ROS Bag\n")

    all_phases,all_phase_times = process_behavior_state(file_data, args.vehicle_namespace)
    if args.list_phases:
        print_phases = set(all_phases)
        print("Behavior Phases:")
        for p in print_phases:
            print(p)
        print()

    if args.list_phase_transitions:
        print("Beahvior Phase Transisions:")
        for p,t in zip(all_phases, all_phase_times):
            print("Phase: {}, Time: {}".format(p,t))
        print()

    if args.list_topics:
        print("ROS Topics:")
        for k in file_data.keys():
            print(k)
        print()

    if not args.no_plot:
        fig_3d = go.Figure()
        fig_2d = go.Figure()
        fig_depth = go.Figure()
        fig_histogram = go.Figure()
        fig_data = go.Figure()

        start_time, end_time = plot_data(file_data, args.vehicle_namespace, data_step=50, fig_3d=fig_3d, fig_2d=fig_2d, fig_depth=fig_depth, fig_data=fig_data, log_data=True, plot_threshold=0.2, phases=args.behavior_phase)
        plot_histogram(fig_histogram, file_data, args.vehicle_namespace, phases=args.behavior_phase)

        external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
        app = dash.Dash(external_stylesheets=external_stylesheets)

        start_end_str = "Start Time: {:.2f} ({:.2f}) End Time: {:.2f} ({:.2f}) Total Time: {:.2f} ({:.2f}) Model End: {:.2f} ({:.2f})".format(start_time/3600, start_time/3600/24, 
                                                                                                                                                                end_time/3600, end_time/3600/24,
                                                                                                                                                                (end_time-start_time)/3600, (end_time-start_time)/3600/24,
                                                                                                                                                                5011200/3600, 5011200/3600/24)

        app.layout = html.Div([
            html.Div([
                    html.P(children=start_end_str, className="info_pane")
                ]),
            html.Div([
                html.Div([
                    dcc.Graph(id='g1', figure=fig_3d, style={'height': '90vh', 'width': '90vh'})
                ], className="six columns"),

                html.Div([
                    dcc.Graph(id='g2', figure=fig_2d, style={'height': '90vh', 'width': '90vh'})
                ], className="six columns"),
            ], className="row"),

            html.Div([
                html.Div([
                    dcc.Graph(id='g3', figure=fig_depth, style={'height': '90vh', 'width': '90vh'})
                ], className="six columns"),


                html.Div([
                    dcc.Graph(id='g4', figure=fig_histogram, style={'height': '90vh', 'width': '90vh'})
                ], className="six columns")
            ], className="row"),

            html.Div([
                html.Div([
                    dcc.Graph(id='g5', figure=fig_data, style={'height': '90vh', 'width': '90vh'})
                ], className="six columns"),
            ], className="row")
        ])

        app.run_server(debug=False, host='0.0.0.0', port=8050)  # Turn off reloader if inside Jupyter

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot Data Run.')
    parser.add_argument("filename", type=str, help="Filename of the rosbag file or parsed pickle file to plot")
    parser.add_argument("-lp", "--list-phases", action='store_true', help="List all beahvior phases at are used")
    parser.add_argument("-lpt", "--list-phase-transitions", action='store_true', help="List all beahvior phases and the transitions associated with them")
    parser.add_argument("-lt", "--list-topics", action='store_true', help="List ros topics that were recorded.")

    parser.add_argument("-np", "--no-plot", action='store_true', help="Don't perform any plotting")

    parser.add_argument("-v", "--vehicle-namespace", type=str, help="Name of the vehicle namespace in the rosbag file")
    parser.add_argument("-p", "--behavior-phase", nargs='+', help="Name of the bahavior phases to plot")

    args = parser.parse_args()
    main(args)