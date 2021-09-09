import plotly.graph_objects as go
import rosbag
import argparse
import numpy as np

import dash
import dash_core_components as dcc
import dash_html_components as html

def plot_histogram(fig_histogram, bag, vehicle_namespace, measurment_type="dye", data_step=1):
    """
    Plot histogram of collected data

    Parameters
    ----------
    fig_histogram : plotly Figure
        Figure to plot data histogram
    bag : Rosbag File
        rosbag file to plot from
    vehicle_namespace : str
        The vehicle namespace for the data to plot
    measurment_type : str
        The measurment type to plot (dye,temp,salt)
    data_step : int
        Every N messages to plot. i.e. 1 is plot every message, 2 is plot every other message.
    """ 

    data = []
    sample_data = []
    current_step = 1
    for topic, msg, t in bag.read_messages(topics=['/' + vehicle_namespace + '/data_broadcaster/data',
                                                   '/' + vehicle_namespace + '/sample/take_sample']):
        if topic == '/' + vehicle_namespace + '/data_broadcaster/data':
            #X and Y flipped because NED reference frame is used in ROS
            if current_step == data_step:
                if measurment_type == "dye":
                    data.append(msg.dye)
                elif measurment_type == "temp":
                    data.append(msg.temp)
                elif measurment_type == "salt":
                    data.append(msg.salt)
                current_step = 1
            else:
                current_step += 1
        elif topic == '/' + vehicle_namespace + '/sample/take_sample':
            sample_data.append(msg.data)

    fig_histogram.add_trace(go.Histogram(x=data))
    for s in sample_data:
        fig_histogram.add_vline(x=s)

def plot_data(bag, vehicle_namespace, measurment_type="dye", data_step=1, fig_3d=None, fig_2d=None, fig_depth=None, fig_data=None, log_data=True, plot_threshold=0):
    """
    Plot data collected by a vehicle.

    Parameters
    ----------
    bag : Rosbag File
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

    start_time = None
    end_time = None
    x = []
    y = []
    z = []
    data = []
    data_time = []
    data_sonar = []

    sample_x = []
    sample_y = []
    sample_z = []
    sample_data = []
    sample_pos_index = []
    sample_time = []

    current_step = 1
    for topic, msg, t in bag.read_messages(topics=['/' + vehicle_namespace + '/data_broadcaster/data',
                                                   '/' + vehicle_namespace + '/sample/take_sample']):
        if topic == '/' + vehicle_namespace + '/data_broadcaster/data':
            if start_time is None:
                start_time = msg.time.to_sec()
            end_time = msg.time.to_sec()
            #X and Y flipped because NED reference frame is used in ROS
            if current_step >= data_step:
                if measurment_type == "dye":
                    new_data = msg.dye
                elif measurment_type == "temp":
                    new_data = msg.temp
                elif measurment_type == "salt":
                    new_data = msg.salt

                if new_data < plot_threshold:
                    new_data = plot_threshold

                #x and y flipped to make north up
                x.append(msg.y)
                y.append(msg.x)
                z.append(-msg.h)
                data_sonar.append(msg.sonarDepth)
                data.append(new_data)
                
                data_time.append((msg.time.to_sec() - start_time)/3600)

                current_step = 1
            else:
                current_step += 1
        elif topic == '/' + vehicle_namespace + '/sample/take_sample':
            if start_time is None:
                start_time = msg.header.stamp.to_sec()
            end_time = msg.header.stamp.to_sec()
            #x and y flipped to make north up
            sample_x.append(msg.location.y)
            sample_y.append(msg.location.x)
            sample_z.append(-msg.location.z)

            sample_data.append(msg.data)
            sample_pos_index.append(len(x))
            sample_time.append((msg.header.stamp.to_sec()-start_time)/3600)


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
    print("Opening ROS Bag")
    bag = rosbag.Bag(args.rosbag)
    print("Opened ROS Bag")

    fig_3d = go.Figure()
    fig_2d = go.Figure()
    fig_depth = go.Figure()
    fig_histogram = go.Figure()
    fig_data = go.Figure()

    start_time, end_time = plot_data(bag, args.vehicle_namespace, data_step=50, fig_3d=fig_3d, fig_2d=fig_2d, fig_depth=fig_depth, fig_data=fig_data, log_data=True, plot_threshold=0.2)
    plot_histogram(fig_histogram, bag, args.vehicle_namespace)

    bag.close()

    external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
    app = dash.Dash(external_stylesheets=external_stylesheets)

    start_end_str = "Start Time: {:.2f} End Time: {:.2f} Total Time: {:.2f} Model End: {:.2f}".format(start_time/3600, end_time/3600, (end_time-start_time)/3600, 5011200/3600)

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
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument("rosbag", type=str, help="Filename of the rosbag file to plot")
    parser.add_argument("-v", "--vehicle_namespace", type=str, help="Name of the vehicle namespace in the rosbag file")

    args = parser.parse_args()
    main(args)