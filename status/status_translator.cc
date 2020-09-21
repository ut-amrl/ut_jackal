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
\file    status_translator.cc
\brief   Translate from GPS to map coordinates
\author  Kavan Sikand, (C) 2020
*/
//========================================================================

#include <stdio.h>

#include <cmath>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"

#include "amrl_msgs/RobofleetStatus.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "jackal_msgs/Status.h"
#include "math/math_util.h"
#include "ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "util/timer.h"
#include "util/helpers.h"

using std::string;

using namespace math_util;

DEFINE_string(status_topic, "/status", "ROS topic for jackal status messsages");
DEFINE_string(localization_topic, "/localization", "ROS topic for amrl localization messsages");
DEFINE_string(output_topic, "/robofleet_status", "ROS topic for Robofleet status messages");
DEFINE_double(timeout, 3.0, "Timeout for resetting fields of status message.");

ros::Publisher status_pub_;
amrl_msgs::RobofleetStatus status_msg_;
double last_status_msg = GetMonotonicTime();
double last_loc_msg = GetMonotonicTime();

void StatusCallback(const jackal_msgs::Status& msg) {
  const bool verbose = FLAGS_v > 0;
  if (verbose) {
    printf("Recieved a status message\n");
    // printf("Status:%d Service:%d Lat,Long:%12.8lf, %12.8lf Alt:%7.2lf",
    //       msg.status.status, msg.status.service,
    //       msg.latitude, msg.longitude, msg.altitude);
  };

  // TODO check status to verify levels are good
  // TODO do more checks e.g. is localization up, etc.
  status_msg_.status = "online";

  //TODO
  status_msg_.is_ok = true;
  status_msg_.battery_level = msg.measured_battery / 100.0;
  last_status_msg = GetMonotonicTime();
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg) {
  const string status = StringPrintf("%s: %.3f,%.3f,%.1f\u00b0", msg.map.c_str(), msg.pose.x, msg.pose.y, RadToDeg(msg.pose.theta));
  status_msg_.location = status;
  last_loc_msg = GetMonotonicTime();
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "status_translator");
  ros::NodeHandle n;
  ros::Subscriber status_sub = n.subscribe(FLAGS_status_topic, 1, &StatusCallback);
  ros::Subscriber loc_sub = n.subscribe(FLAGS_localization_topic, 1, &LocalizationCallback);
  status_pub_ =  n.advertise<amrl_msgs::RobofleetStatus>(FLAGS_output_topic, 1);

  double curr_time;
  ros::Rate loop_rate(10);
  while (ros::ok()) {

    curr_time = GetMonotonicTime();
    
    // haven't heard from the robot in too long!
    if (curr_time - last_status_msg > FLAGS_timeout) {
      status_msg_.status = "unknown";
      status_msg_.is_ok = false;
      status_msg_.battery_level = 0.0;
    }

    if (curr_time - last_loc_msg > FLAGS_timeout) {
      status_msg_.location = "";
    }

    status_pub_.publish(status_msg_);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}