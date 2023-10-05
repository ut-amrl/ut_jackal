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
