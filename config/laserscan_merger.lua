laserscan_merger = {
  scan_topics = {
    "/ouster_laserscan",
    "/kinect_laserscan",
  };
  merged_scan_topic = "/merged_laserscan";
  target_frame = "base_link";
  time_threshold = 0.05;

  angle_min = -3.14159274101;
  angle_max = 3.14159274101;
  num_ranges = 360;
  range_min = 0.1;
  range_max = 30.0;
};
