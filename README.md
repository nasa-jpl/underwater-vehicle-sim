# ROS Underwater Simulation
Extraterrestrial hydrothermal vent search

## Docker

### Dockerfile
A Dockerfile is included to setup the environemnt.  The Dockerfile has multiple stages. The first stage `ros-underwater-sim-external-dependencies` sets up just the external dependencies, the second stage `ros-underwater-sim-all-dependencies` adds internal dependencies in the form of the `ocean-model-interface`, the third stage `ros-underwater-sim` copies over and builds the source code for the simulation. 

### Compose Scripts

Two compose scripts are provided for a development environment and a deployment environment


#### Development Environment

`compose-dev.yaml` uses `ros-underwater-sim-all-dependencies` as the base and bind mounts the source code for development. The source code is mounted in `/ros_workspace`

Build and start the container for the development environment.

`docker compose -f compose-dev.yaml up -d --build`

Attach to the container via a terminal.

`docker exec -it ros-underwater-sim-dev bash`

#### Deployment Environment

`compose.yaml` uses `ros-underwater-sim` as the base with the source code copied in the image. Currently this does not start the simulation environment as it is common to need to restart the environment or manually process data in the container after a run is complete.

Build and start the container for the deployment environment.

`docker compose -f compose.yaml up -d --build`

Attach to the container via a terminal.

`docker exec -it ros-underwater-sim bash`


### Mounted Volumes
In both setups, there are two volumes that are mounted automatically.

`./config:/config`: For providing custom configuration files such as launch files or model files to the simulation
`./docker_data:/docker_data`: For outputting and saving simulation data.

### Extending the Docker Images

When developing new software that uses the simulation environment the easiest way to set it up is to build of off the docker image produced by the end stage of the Dockerfile (`ros-underwater-sim`). This places the simulation and the new software all in one container with simplifies coordination for starting and stopping the simulation, creating launch files, and saving results.

Running `docker build . -t ros-underwater-sim` will build the image and name it `ros-underwater-sim`. The ros workspace is created at `/ros_workspace` and contains the packages needed for the ros-underwater-sim. Other ROS packages can be added to this folder and built alongside the simulation environment.

Then image containing the simulation environment can be used directly in a compose script (see `docker_examples/compose-extend-example.yaml`) or it can be used as the base for other Dockerfiles containing the setup for the development environment for the new software.

The alternative is to run the ros simulation and the other ros nodes in two separate services, but this complicates coordination of all the ROS nodes.

## Build

The simulation can be built by running the following command in the `/ros_workspace` directory.

`catkin_make`

Then run the following command so ros knows about the packages

`source devel/setup.bash`

## Unit Tests

Run the following in `/ros_workspace`

`catkin_make run_tests`

`catkin_test_results`

## Using the Simulation

### Running
`roslaunch <path/to/launch/file>`

If you want to output a rosbag you can place the following in the launch file, replacing `<dir_name>` with some subdirectory to store rosbags from this particular launch file. This should then be available outside of the docker container via the mounted directory.

```
 <node pkg="rosbag" type="record" name="rosbag_record"
   args='record -o /docker_data/<dir_name> -e "/v1/(.*)" /rosout'
   if="$(arg record_rosbag)" />   
```

### Services

**/<vehicle_namespace>/go_to_xy : GoToXY.srv**

Commands the vehicle to go to an xy waypoint or to stop xy movement.  This will override any previous commands controlling the XY vehicle movement.
```
float64 x : X coordination of the target waypoint in meters from origin
float64 y : Y coordination of the target waypoint in meters from origin
float64 xLinearVelocity : The target linear velocity of the vehicle when going to the waypoint
float64 zAngularVelocity : The target rotational velocity when going to the waypoint
bool enable : If true target the provided xy waypoint, if false stop xy movement.
---
# No service result information
```

**/<vehicle_namespace>/go_to_z : GoToZ.srv**

Commands the vehicle to go to a specified depth or to stop Z movement
```
float64 depth : The depth to target in meters
float64 zLinearVelocity : The target vertical velocity when going to the depth
bool holdDepth : If true the vehicle will actively hold this depth
bool enable : If true target the provided depth, if false stop z movement.
---
# No service result information
```

**/<vehicle_namespace>/follow_heading : FollowHeading.srv**

Commands the vehicle to follow a specified heading. This will override any previous commands controlling the XY vehicle movement.
```
float64 heading : The target heading for the vehicle to follow in radians
float64 xLinearVelocity : The target linear velocity of the vehicle when following the heading
float64 zAngularVelocity : The target rotational velocity when following the heading
bool enable : If true target the provided heading, if false stop xy movement.
---
# No service result information
```

### Topics

**/<vehicle_namespace>/primary_navigation : nav_msgs::Odometry**

The primary source of navigation information for the vehicle. This is the navgiation source used by the propulsion control system. The primary_navigation topic is controlled via _remap_ in the launch xml file.

**/<vehicle_namespace>/prop_state : PropulsionControllerState.msg**

This topic is published by the propulsion controller and contains the current state of the controller.

```
Header header : The header containing the message timestamp.

bool xyEnable : True if the xy (horizontal) propulsion system is enabled, false otherwise
bool xyComplete : True if the latest xy command has been completed, false if it is still active
int64 xySeqNum : A sequence number that increments each time an xy command is completed.
float64 x : The x target of the current xy command
float64 y : The y target of the current xy command

bool zEnable : True if the z (vertical) propulsion system is enabled, false otherwise
bool zComplete : True if the latest z (depth) command has been completed, false if it is still active
int64 zSeqNum : A sequence number that increments each time an z (depth) command is completed.
float64 z : The depth target of the current z command.
bool holdDepth : True if the current command is set to actively hold depth, false otherwise
```

**/<vehicle_namespace>/<data_broadcaster_module_name>/data : PropulsionControllerState.msg**

This topic will be published if a `DataBroadcaster` module is defined in the vehicle sim node. The example launch files at `config/launch` show this. In those examples the topic name is `/v1/data_broadcaster/data`. This topic contains sensor data retreived from the ocean model that is being used as well as the ground truth vehicle position information.

```
string name : The name of the vehicle
float64 x : The x position of the vehicle
float64 y : The y position of the vehicle
float64 h : The h (height) position of the vehicle
float64 sonarDepth : The distance from the vehicle to the seafloor
time time : The time of the data
float64 temp : The temperature from the model
float64 salt : The salinity from the model
float64 dye : The neutrally buoyant tracer dye from the model
float64 u : The u current from the model
float64 v : The v current form the model
```

**/<vehicle_namespace>/<usbl_module_name>/data : USBL.msg**

This topic will be published if a `USBL` module is defined in the vehicle sim node. The example launch files at `config/launch` show this. This topic contains data from a simulated usbl sensor

```
Header header : The header containing the timestamp of the data

string name : The name of the vehicle

float64 beacon_x : The ground truth beacon x location
float64 beacon_y : The ground truth beacon y location
float64 beacon_z : The ground truth beacon z location

float64 range : The range between the two partso f the USBL system. This can either be a standard or inverted configuration.
float64 bearing : The bearing between the two partso f the USBL system. This can either be a standard or inverted configuration.

float64[4] range_bearing_covariance : The covariance of the measurement
```

**/<vehicle_namespace>/<dvl_module_name>/data : DVL.msg**

This topic will be published if a `DVL` module is defined in the vehicle sim node. The example launch files at `config/launch` show this. This topic contains data from a simulated dvl sensor

```
Header header : The header containing the timestamp of the data
string name : The vehicle name
geometry_msgs/Vector3 velocity : Measured velocity [m/s] in sensor frame
float64[9] velocity_covariance : Row major, xyz axes
float64 range : Bottom range, only valid if the velocity_reference is VELOCITY_REFERENCE_BOTTOM

uint8 VELOCITY_REFERENCE_UNKNOWN = 0
uint8 VELOCITY_REFERENCE_WATER = 1
uint8 VELOCITY_REFERENCE_BOTTOM = 2
uint8 velocity_reference : The reference for the velocity measurement. Should be one of the above constants
```


### Configuration

See `config/launch/example_vehicle.launch` or `config/launch/example_vehicle_pid.launch` to see example configurations for the vehicle sim nodes.

#### Ocean Model
An ocean model can be specified in the configuration file to be loaded and used to populate the vehicle sensor information. The https://github.com/nasa-jpl/ocean-model-interfaces.git library is used to manage the access the model data. The `void loadModelLocal(ros::NodeHandle& nhPriv)` function in `UnderwaterVehicleSimNode.cpp` is used to load the model and looking at that function can show the parameter used for each model. All models in the `ocean-model-interface` library are supported. Those are currently:

```
fvcom
geodetic_grid (e.g. ROMS that has been processed into a regular geodetic grid)
constant
linear
front
```

## ROS Bag Visualization
To visualize the data saved in the rosbag simulation do the following: 

Within the docker container run the following to turn the rosbag into a python pickle file so we can use it without ROS installed outside of the docker container. This is where the `/docker_data` volume becomes useful.

`python3 /ros_workspace/src/ros-underwater-sim/src/ros_underwater_sim_utilities/src/scripts/rosbag_processing/parse_bag.py -o <output_directory> <rosbag_input_file>`

Run the following outside of the docker container to plot the vehicle path and data using Plotly. If other analysis is needed the script provides an example of how to use the pickle files generated from the rosbags.

`python ./ros-underwater-sim/src/ros_underwater_sim_utilities/src/scripts/plots/post_processing_plot.py -v v1 <python_pickle_file>`


# Copyright

Copyright 2026, by the California Institute of Technology. ALL RIGHTS RESERVED. United States Government Sponsorship acknowledged. Any commercial use must be negotiated with the Office of Technology Transfer at the California Institute of Technology.

This software may be subject to U.S. export control laws. By accepting this software, the user agrees to comply with all applicable U.S. export laws and regulations. User has the responsibility to obtain export licenses, or other export authority as may be required before exporting such information to foreign countries or providing access to foreign persons.

# Point of Contact

Andrew Branch - andrew.branch@jpl.nasa.gov
