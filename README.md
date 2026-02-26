# ROS Underwater Simulation
Extraterrestrial hydrothermal vent search

## Docker

### Dockerfile
A Dockerfile is included to setup the environemnt.  The Dockerfile has multiple stages. The first stage `ros-underwater-sim-external-dependencies` sets up just the external dependencies, the second stage `ros-underwater-sim-all-dependencies` adds internal dependencies in the form of the `ocean-model-interface`, the third stage `ros-underwater-sim` copies over and builds the source code for the simulation. 

The libraries needed for using numerical ocean models in the simulation are included in `ros-underwater-sim-dependencies` .

### Compose Scripts

#### Deployment Environment

`compose.yaml` uses `ros-underwater-sim` as the base with the source code copied in the image. Currently this does not start the simulation environment as it is common to need to restart the environment or manually process data in the container after a run is complete.

Build and start the container for the deployment environment.

`docker compose -f compose.yaml up -d --build`

Attach to the container via a terminal.

`docker exec -it ros-underwater-sim bash`


#### Development Environment

`compose-dev.yaml` uses `ros-underwater-sim-dependencies` as the base and bind mounts the source code for development. 

Build and start the container for the development environment.

`docker compose -f compose-dev.yaml up -d --build`

Attach to the container via a terminal.

`docker exec -it ros-underwater-sim-dev bash`

### Mounted Volumes
In both setups, there are two volumes that are mounted automatically.

`./config:/config`: For providing custom configuration files such as launch files or model files to the simulation
`./docker_data:/docker_data`: For outputting and saving simulation data.

### Extending the Docker Images

When developing new software that uses the simulation environment the easiest way to set it up is to build of off the docker image produced by the end stage of the Dockerfile (`ros-underwater-sim`). Running `docker compose -f compose-dev.yaml up --build` will build the image and name it `ros-underwater-sim`. Then this can be used as the base for other Dockerfiles containing the setup for the environment for the new software. This puts the new code being developed in the same container as the simulation environment, which is nice for development as it makes it easy to start and stop the simulation environment, create launch files that start both the simulation and other ROS nodes, and save results. The ros workspace is created at `/ros_workspace` and contains the packages needed for the ros-underwater-sim. Other ROS packages can be added to this folder and built alongside the simulation environment. The base image also includes all the needed ROS dependencies so those do not need to be setup again.

The alternative is to run the ros simulation and the other ros nodes in two separate services, but this complicates coordination of all the ROS nodes.

## Build

The simulation can be built by running the following command in the `/ros_workspace` directory.

`catkin_make`

Then run the following command so ros knows about the packages

`source devel/setup.bash`

## Unit Tests

`catkin_make run_tests`

`catkin_test_results`

# Copyright

Copyright 2025, by the California Institute of Technology. ALL RIGHTS RESERVED. United States Government Sponsorship acknowledged. Any commercial use must be negotiated with the Office of Technology Transfer at the California Institute of Technology.

This software may be subject to U.S. export control laws. By accepting this software, the user agrees to comply with all applicable U.S. export laws and regulations. User has the responsibility to obtain export licenses, or other export authority as may be required before exporting such information to foreign countries or providing access to foreign persons.

# Point of Contact

Andrew Branch - andrew.branch@jpl.nasa.gov
