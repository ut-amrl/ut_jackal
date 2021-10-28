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
\file    autonomy_arbiter.cc
\brief   Gating for autonomy.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>

#include "geometry_msgs/Twist.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"

#include "shared_lib/util/helpers.h"
#include "shared_lib/util/timer.h"
#include "config_reader/config_reader.h"

// Verbose level flag.
DECLARE_int32(v);
DEFINE_string(config, "config/autonomy_arbiter.lua", "config file location");

// The source topic to read autonomous commands from.
CONFIG_STRING(src_topic, "AutonomyArbiterParameters.src_topic");
// The destination topic to write autonomous commands to.
CONFIG_STRING(dest_topic, "AutonomyArbiterParameters.dest_topic");
// The destination topic to publish autonomy status to.
CONFIG_STRING(status_topic, "AutonomyArbiterParameters.status_topic");
// The topic of the Joystick controller.
CONFIG_STRING(joystick_topic, "AutonomyArbiterParameters.joystick_topic");
// The button used to indicate start of autonomous operation.
CONFIG_UINT(start_btn_idx, "AutonomyArbiterParameters.start_btn_idx");
CONFIG_STRINGLIST(recording_topics, "AutonomyArbiterParameters.recording_topics");
CONFIG_STRING(record_directory, "AutonomyArbiterParameters.record_directory");

// Drive publisher.
ros::Publisher drive_pub_;

// Status publisher.
ros::Publisher status_pub_;

// Enable drive.
bool enable_drive_ = false;

struct AutonomyArbiterParameters {
  // The source topic to read autonomous commands from.
  std::string src_topic;

  // The destination topic to write autonomous commands to.
  std::string dest_topic;
  
  // The destination topic to publish autonomy status to.
  std::string status_topic;

  // The topic of the Joystick controller.
  std::string joystick_topic;

  // The button used to indicate start of autonomous operation.
  uint64_t start_btn_idx;

  std::vector<std::string> recording_topics;

  std::string record_directory;

  // Default constructor, just set defaults.
  AutonomyArbiterParameters() :
      src_topic("navigation/cmd_vel"),
      dest_topic("cmd_vel"),
      status_topic("autonomy_arbiter/enabled"),
      joystick_topic("bluetooth_teleop/joy"),
      start_btn_idx(0),
      recording_topics({}),
      record_directory("/data/") {}
};

void LoadConfig(AutonomyArbiterParameters* params) {
  #define REAL_PARAM(x) CONFIG_DOUBLE(x, "AutonomyArbiterParameters."#x);
  #define NATURALNUM_PARAM(x) CONFIG_UINT(x, "AutonomyArbiterParameters."#x);
  #define STRING_PARAM(x) CONFIG_STRING(x, "AutonomyArbiterParameters."#x);
  #define STRINGLIST_PARAM(x) CONFIG_STRINGLIST(x, "AutonomyArbiterParameters."#x);
  #define BOOL_PARAM(x) CONFIG_BOOL(x, "AutonomyArbiterParameters."#x);
  STRING_PARAM(src_topic);
  STRING_PARAM(dest_topic);
  STRING_PARAM(joystick_topic);
  STRING_PARAM(status_topic);
  NATURALNUM_PARAM(start_btn_idx);
  STRINGLIST_PARAM(recording_topics);

  config_reader::ConfigReader reader({FLAGS_config});
  params->src_topic = CONFIG_src_topic;
  params->dest_topic = CONFIG_dest_topic;
  params->joystick_topic = CONFIG_joystick_topic;
  params->status_topic = CONFIG_status_topic;
  params->start_btn_idx = CONFIG_start_btn_idx;
  params->recording_topics = CONFIG_recording_topics;
}

void DriveCallback(const geometry_msgs::Twist& msg) {
  if (enable_drive_) {
    drive_pub_.publish(msg);
  } else {
    // TODO: Ramp down to 0.
  }
}

void JoystickCallback(const sensor_msgs::Joy& msg) {
  static const double kDebounceTimeout = 0.5;
  static double t_debounce_start_ = 0;
  if (GetMonotonicTime() < t_debounce_start_ + kDebounceTimeout) {
    return;
  }
  bool need_to_debounce = false;
  if (enable_drive_) {
    for (size_t i = 0; i < msg.buttons.size(); ++i) {
      if (msg.buttons[i] == 1) {
        enable_drive_ = false;
        if (FLAGS_v > 0) {
          printf("Drive disabled.\n");
        }
        need_to_debounce = true;
      }
    }
  } else {
    bool start_pressed = false;
    bool all_else_unpressed = true;
    for (size_t i = 0; i < msg.buttons.size(); ++i) {
      if (i == CONFIG_start_btn_idx) {
        start_pressed = msg.buttons[i] == 1;
      } else {
        all_else_unpressed = all_else_unpressed && msg.buttons[i] == 0;
      }
    }
    enable_drive_ = start_pressed && all_else_unpressed;
    if (enable_drive_) {
      need_to_debounce = true;
      if (FLAGS_v > 0) {
        printf("Drive enabled.\n");
      }
    }
  }
  static bool recording = false;
  // See if recording should start.
  // Stop with red circle.
  if (recording && msg.buttons[1] == 1) {
      recording = false;
      if (system("killall rosbag") != 0) {
        printf("Unable to kill rosbag!\n");
      } else {
        printf("Stopped recording rosbag.\n");
      }
      need_to_debounce = true;
  } else if (!recording && msg.buttons[2] == 1) {
    // Start with green triangle.
    printf("Starting recording rosbag...\n");
    std::string record_cmd = "rosbag record ";
    record_cmd += "-o " + CONFIG_record_directory + " ";
    for (std::string cmd : CONFIG_recording_topics) {
      record_cmd += cmd + " ";
    }
    record_cmd += "&";

    if (system(record_cmd.c_str()) != 0) {
      printf("Unable to record\n");
    } else {
      printf("Started recording rosbag.\n");
      recording = true;
    }
    need_to_debounce = true;
  }

  if (need_to_debounce) {
    t_debounce_start_ = GetMonotonicTime();
  }
  std_msgs::Bool status_msg;
  status_msg.data = enable_drive_;
  status_pub_.publish(status_msg);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  // Initialize ROS.
  ros::init(argc, argv, "autonomy_arbiter", ros::init_options::NoSigintHandler);

  AutonomyArbiterParameters params;
  LoadConfig(&params);

  printf("Starting autonomy arbiter");
  if (FLAGS_v > 0) {
    printf("Autonomy arbiter\n");
    printf("Source topic: %s\n", CONFIG_src_topic.c_str());
    printf("Destination topic: %s\n", CONFIG_dest_topic.c_str());
    printf("Joystick topic: %s\n", CONFIG_joystick_topic.c_str());
  }
  ros::NodeHandle n;
  ros::Subscriber joystick_sub =
      n.subscribe(CONFIG_joystick_topic, 1, &JoystickCallback);
  ros::Subscriber drive_sub =
      n.subscribe(CONFIG_src_topic, 1, &DriveCallback);
  drive_pub_ = n.advertise<geometry_msgs::Twist>(CONFIG_dest_topic, 1);
  status_pub_ = n.advertise<std_msgs::Bool>(CONFIG_status_topic, 1);
  ros::spin();
  return 0;
}
