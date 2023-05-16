//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    pointcloud_to_laserscan_main.cc
\brief   A point cloud to laserscan convert that actually works without
         crashing or throwing buffer overflow errors.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "config_reader/config_reader.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "math/math_util.h"
#include "util/timer.h"
#include "ros/ros_helpers.h"

using math_util::DegToRad;
using math_util::RadToDeg;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
using std::string;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector3f;
using math_util::Sq;
using math_util::DegToRad;

DECLARE_string(helpon);
DECLARE_int32(v);

sensor_msgs::LaserScan laser_msg_;
float max_height_ = FLT_MAX;
float min_height_ = -FLT_MAX;
float min_sq_range_ = 0;
float max_sq_range_ = FLT_MAX;
string laser_topic_ = "scan";
string pointcloud_topic_ = "pointcloud";

static const Eigen::Affine3f frame_tf_ =
      Eigen::Translation3f(0, 0, 0.85) *
      Eigen::AngleAxisf(0.0, Vector3f::UnitX());
const std::string target_frame_("base_link");

ros::Publisher scan_publisher_;


void PointcloudCallback(const sensor_msgs::PointCloud2& msg) {
  static CumulativeFunctionTimer function_timer_(__FUNCTION__);
  CumulativeFunctionTimer::Invocation invoke(&function_timer_);
  if (FLAGS_v > 1) {
    printf("PointCloud2 message, t=%f\n", msg.header.stamp.toSec());
  }
  laser_msg_.header = msg.header;
  laser_msg_.header.frame_id = target_frame_;

  for (float& r : laser_msg_.ranges) {
    r = FLT_MAX;
  }
  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float>
      iter_x(msg, "x"),
      iter_y(msg, "y"), iter_z(msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!isfinite(*iter_x) || !isfinite(*iter_y) || !isfinite(*iter_z)) {
      continue;
    }
    const Vector3f p = frame_tf_ * Vector3f(*iter_x, *iter_y, *iter_z);

    if (p.z() > max_height_ || p.z() < min_height_) {
      continue;
    }

    const float sq_range = Sq(p.x()) + Sq(p.y());
    if (sq_range < min_sq_range_ || sq_range > max_sq_range_) continue;
    const float angle = atan2(p.y(), p.x());

    const size_t idx = static_cast<size_t>(floor(
        (angle - laser_msg_.angle_min) / laser_msg_.angle_increment));
    if (laser_msg_.ranges[idx] > sq_range) {
      laser_msg_.ranges[idx] = sq_range;
    }
  }

  for (float& r : laser_msg_.ranges) {
    if (r < FLT_MAX) {
      r = sqrt(r);
    } else {
      r = 0;
    }
  }
  scan_publisher_.publish(laser_msg_);
}

DEFINE_string(config, "config.lua", "Configuration file");

void LoadConfig() {
  CONFIG_FLOAT(range_min, "pointcloud_to_laser.range_min");
  CONFIG_FLOAT(range_max, "pointcloud_to_laser.range_max");
  CONFIG_FLOAT(angle_min, "pointcloud_to_laser.angle_min");
  CONFIG_FLOAT(angle_max, "pointcloud_to_laser.angle_max");
  CONFIG_FLOAT(height_min, "pointcloud_to_laser.height_min");
  CONFIG_FLOAT(height_max, "pointcloud_to_laser.height_max");
  CONFIG_FLOAT(num_ranges, "pointcloud_to_laser.num_ranges");
  CONFIG_STRING(pointcloud_topic_, "pointcloud_to_laser.pointcloud_topic");
  CONFIG_STRING(laser_topic_, "pointcloud_to_laser.laser_topic");

  config_reader::ConfigReader reader({FLAGS_config});

  laser_msg_.angle_min = CONFIG_angle_min;
  laser_msg_.angle_max = CONFIG_angle_max;
  laser_msg_.angle_increment =
      (CONFIG_angle_max - CONFIG_angle_min) / CONFIG_num_ranges;
  laser_msg_.range_min = CONFIG_range_min;
  laser_msg_.range_max = CONFIG_range_max;
  laser_msg_.ranges.resize(CONFIG_num_ranges);
  min_height_ = CONFIG_height_min;
  max_height_ = CONFIG_height_max;
  min_sq_range_ = Sq(CONFIG_range_min);
  max_sq_range_ = Sq(CONFIG_range_max);
  laser_topic_ = CONFIG_laser_topic_;
  pointcloud_topic_ = CONFIG_pointcloud_topic_;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  LoadConfig();
  ros::init(argc, argv, "pointcloud_to_laserscan");
  ros::NodeHandle n;
  ros::Subscriber pointcloud_sub =
      n.subscribe(pointcloud_topic_, 1, &PointcloudCallback);
  scan_publisher_ = n.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
  ros::spin();
  return 0;
}
