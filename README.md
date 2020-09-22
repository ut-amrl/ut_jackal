# ut_jackal
A single combined repository of all code needed to run the UT Campus Jackal

## Dependencies
1. ROS
1. [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
1. Lua5.1
1. glog
1. gflags
1. Eigen3
1. Ceres-Solver
1. jackal_msgs ros package

## Build
1. Add the repo path to the `ROS_PACKAGE_PATH` environment variable
1. Run `make [-j]`

## Usage
after a successful `make`, and setting up configs for the appropriate subrepos (enml + graph_navigation) simply do:
`roslaunch ut_jackal autonomy.launch`

Parameters can be changed in the launch file, or in the configs of the individual robots.
