/*!
  \file    laserscan_merger_main.cc
  \brief   Merge multiple laser scans from different sensors into one scan
  \author  Dongmyeong Lee
*/
#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <mutex>

#include "config_reader/config_reader.h"
#include "sensor_msgs/LaserScan.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "math/math_util.h"

using std::string;
using std::vector;
using math_util::Sq;

DECLARE_int32(v);

vector<string> scan_topics_ = {"ouster_laserscan"};
std::map<string, std::pair<sensor_msgs::LaserScan::ConstPtr,
                           ros::Time>> scan_buffers_;
std::mutex buffer_mutex_;
float time_threshold_ = 0.1;
float min_sq_range_ = 0;
float max_sq_range_ = FLT_MAX;

sensor_msgs::LaserScan merged_scan_msg_;
string merged_scan_topic_ = "merged_laserscan";
string target_frame_ = "base_link";

ros::Publisher merged_scan_pub_;


bool areScansSynchronized() {
  float min_time = FLT_MAX;
  float max_time = -FLT_MAX;
  for (const auto& scan_buffer : scan_buffers_) {
    float time = scan_buffer.second.second.toSec();
    if (time < min_time) min_time = time;
    if (time > max_time) max_time = time;
  }
  return max_time - min_time < time_threshold_;
}

void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg,
                       const string& topic) {
  // Store the scan in the buffer
  ros::Time received_time = ros::Time::now();

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  scan_buffers_[topic] = std::make_pair(msg, received_time);

  // Check if all scans are received
  if (scan_buffers_.size() < scan_topics_.size() ||
      !areScansSynchronized()) {
    return;
  }

  // Merge the scans
  merged_scan_msg_.header.stamp = ros::Time::now();
  merged_scan_msg_.header.frame_id = target_frame_;

  for (float& r : merged_scan_msg_.ranges) {
    r = FLT_MAX;
  }

  for (const auto& scan_buffer : scan_buffers_) {
    for (size_t i = 0; i < scan_buffer.second.first->ranges.size(); ++i) {
      const float sq_range = Sq(scan_buffer.second.first->ranges[i]);
      if (sq_range < min_sq_range_ || sq_range > max_sq_range_) continue;
      const float angle = scan_buffer.second.first->angle_min +
                          i * scan_buffer.second.first->angle_increment;
      const size_t idx = static_cast<size_t>(round(
          (angle - merged_scan_msg_.angle_min) / 
          merged_scan_msg_.angle_increment));
      if (idx >= merged_scan_msg_.ranges.size()) continue;
      if (merged_scan_msg_.ranges[idx] > sq_range) {
        merged_scan_msg_.ranges[idx] = sq_range;
      }
    }
  }

  for (float& r : merged_scan_msg_.ranges) {
    if (r < FLT_MAX) {
      r = sqrt(r);
    } else {
      r = 0;
    }
  } // End of merging

  merged_scan_pub_.publish(merged_scan_msg_);
  scan_buffers_.clear();
}

DEFINE_string(config, "config.lua", "Configuration file");

void LoadConfig() {
  CONFIG_FLOAT(angle_min, "laserscan_merger.angle_min");
  CONFIG_FLOAT(angle_max, "laserscan_merger.angle_max");
  CONFIG_INT(num_ranges, "laserscan_merger.num_ranges");
  CONFIG_FLOAT(range_min, "laserscan_merger.range_min");
  CONFIG_FLOAT(range_max, "laserscan_merger.range_max");
  CONFIG_STRINGLIST(scan_topics, "laserscan_merger.scan_topics");
  CONFIG_STRING(merged_scan_topic, "laserscan_merger.merged_scan_topic");
  CONFIG_STRING(target_frame, "laserscan_merger.target_frame");
  CONFIG_FLOAT(time_threshold, "laserscan_merger.time_threshold");

  config_reader::ConfigReader reader({FLAGS_config});
  
  merged_scan_msg_.angle_min = CONFIG_angle_min;
  merged_scan_msg_.angle_max = CONFIG_angle_max;
  merged_scan_msg_.angle_increment =
      (CONFIG_angle_max - CONFIG_angle_min) / CONFIG_num_ranges;
  merged_scan_msg_.range_min = CONFIG_range_min;
  merged_scan_msg_.range_max = CONFIG_range_max;
  merged_scan_msg_.ranges.resize(CONFIG_num_ranges);
  min_sq_range_ = Sq(CONFIG_range_min);
  max_sq_range_ = Sq(CONFIG_range_max);
  
  scan_topics_ = CONFIG_scan_topics;
  merged_scan_topic_ = CONFIG_merged_scan_topic;
  target_frame_ = CONFIG_target_frame;
  time_threshold_ = CONFIG_time_threshold;
}

int main(int argc , char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  LoadConfig();

  ros::init(argc, argv, "laserscan_merger");
  ros::NodeHandle n;

  vector<ros::Subscriber> scan_subs_;
  for (const auto& scan_topic : scan_topics_) {
    scan_subs_.push_back(n.subscribe<sensor_msgs::LaserScan>(scan_topic, 1,
        [scan_topic](const sensor_msgs::LaserScan::ConstPtr& msg) {
          LaserScanCallback(msg, scan_topic);
        }));
  }
  merged_scan_pub_ = n.advertise<sensor_msgs::LaserScan>(
      merged_scan_topic_, 1);
  ros::spin();
  return 0;
}
