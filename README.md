# ut_jackal
A single combined repository of all code needed to run the UT Campus Jackal

## Dependencies
1. ROS
1. [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
1. Lua5.1
1. glog
1. gflags
1. Eigen3
1. [Ceres-Solver](http://ceres-solver.org/installation.html#linux)
1. jackal_msgs ros package

## Build
1. Clone the repository and add it to the `ROS_PACKAGE_PATH` environment variable. Also, add `ut_jackal/graph_navigation` path to the `ROS_PACKAGE_PATH` environment variable.
1. Run `git submodule update --init --recursive` from within `ut_jackal` folder.
1. Run `make [-j]`

## Usage
After a successful `make`, the configuration for different repos (enml, graph_navigation, etc) can be changed in the `config/` folder. Once done, simply run:
```
roslaunch ut_jackal autonomy.launch
```

It is recommended to run this in a screen. To do so, run 
```
screen -S <name of screen, ex. 'autonomy'>
roslaunch ut_jackal autonomy.launch
<ctrl+A, then ctrl+D to leave the screen>
```

To resume the screen, run `screen -r <screen name>`


## Basic Demos
### Waypoint Loop

To run through a loop of waypoints, ensure the autonomy stack (previous section) is running. 

If not using in an empty map, you must first localize the robot. If not already localized, set the current pose via [robofleet](robofleet.csres.utexas.edu) or using the `/set_pose` topic. 

Then run the waypoint loop script as follows:
```
cd ~/amrl/ut_jackal/scripts/
python3 waypoint_loop.py --stop_time <number of seconds to stop at each waypoint> --map <name of the waypoints list to run through>
```
The list of waypoints is in the `waypoint_loop.py` script. You can add or modify entries in the waypoint list if you want a different set of waypoints. Note that the map names in this script may not match with the map names in `amrl_maps`. The format of each waypoint is `pos_x, pos_y, quaternion_z, quaternion_w` and these are expressed relative to the map that the localization is currently using for the robot. 

If you want to run through a triangle of points relative to the robot's current position, we've created a utility script that does so with an empty map. To run this, use:
`./amrl/ut_jackal/scripts/loop_from_curr_loc.sh`

Note that because this is configured to run without a map, the waypoint locations may drift over time. 

You may also want to run this in a screen (see above). 


