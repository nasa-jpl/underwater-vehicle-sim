import plotly.graph_objects as go
import argparse
import numpy as np

import dash
import dash_core_components as dcc
import dash_html_components as html
import json

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

def plot_data(bag, vehicle_namespace, measurment_type="dye", data_step=1, fig_3d=None, fig_2d=None, fig_depth=None, fig_data=None, log_data=True, plot_threshold=0):
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
    """

    start_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][0]
    end_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][-1]

    x = bag['/' + vehicle_namespace + '/data_broadcaster/data']["y"][::data_step]
    y = bag['/' + vehicle_namespace + '/data_broadcaster/data']["x"][::data_step]
    z = [-h for h in bag['/' + vehicle_namespace + '/data_broadcaster/data']["h"][::data_step]]
    data = [max(plot_threshold,d) for d in bag['/' + vehicle_namespace + '/data_broadcaster/data'][measurment_type][::data_step]]
    
    data_time = bag['/' + vehicle_namespace + '/data_broadcaster/data']["time"][::data_step]
    data_sonar = bag['/' + vehicle_namespace + '/data_broadcaster/data']["sonar_depth"][::data_step]


    if '/' + vehicle_namespace + '/sample/take_sample' in bag:
        sample_x = []
        sample_y = []
        sample_z = []
        for x_loc,y_loc,z_loc in bag['/' + vehicle_namespace + '/sample/take_sample']["location"]:
            sample_x.append(y_loc)
            sample_y.append(x_loc)
            sample_z.append(-z_loc)

        sample_data = bag['/' + vehicle_namespace + '/sample/take_sample']["data"]
        sample_time = bag['/' + vehicle_namespace + '/sample/take_sample']["time"]
    else:
        sample_data = []
        sample_time = []
        sample_x = []
        sample_y = []
        sample_z = []

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
        
        fig_2d.update_layout(yaxis=dict(scaleanchor='x', scaleratio=1)) # Makes 1 data unit on y equal to 1 data unit on x


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
    file_data = json.load(open(args.filename,"rb"))
    print("Opened and Parsed ROS Bag\n")

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

        start_time, end_time = plot_data(file_data, args.vehicle_namespace, data_step=10, fig_3d=fig_3d, fig_2d=fig_2d, fig_depth=fig_depth, fig_data=fig_data, log_data=True, plot_threshold=0)

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
    parser.add_argument("-lt", "--list-topics", action='store_true', help="List ros topics that were recorded.")

    parser.add_argument("-np", "--no-plot", action='store_true', help="Don't perform any plotting")

    parser.add_argument("-v", "--vehicle-namespace", type=str, help="Name of the vehicle namespace in the rosbag file")

    args = parser.parse_args()
    main(args)