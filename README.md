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

## Running the Simulation

`roslaunch <path/to/launch/file>`

If you want to output a rosbag you can place the following in the launch file, replacing `<dir_name>` with some subdirectory to store rosbags from this particular launch file. This should then be available outside of the docker container via the mounted directory.

```
 <node pkg="rosbag" type="record" name="rosbag_record"
   args='record -o /docker_data/<dir_name> -e "/v1/(.*)" /rosout'
   if="$(arg record_rosbag)" />   
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
