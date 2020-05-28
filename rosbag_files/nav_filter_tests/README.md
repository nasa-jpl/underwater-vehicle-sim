# Navigation Filter ROSBag Test Files
The files in this directory tree are used to test navigation filter performance.

Each directory contains a single run. The rosbag and parameters that are used for the run are included.

## Playback ROSBag Files
A rosbag file can be played back by using `rosbag play --clock <filename>`. The `use_sim_time` ros parameter should be set so the rostime is taken from /clock. A ROSSimNavigationNode can be run with the desired filters. The output of those filter and any other data of interest can then be recorded in a rosbag for plotting.

## Recording New ROSBag Files
rosbag recording command: `rosbag record v1/imu/data v1/dvl/data v1/usbl/data v1/depth/data v1/nav_filters/true_nav /clock`
