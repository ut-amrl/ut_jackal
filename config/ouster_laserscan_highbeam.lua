pointcloud_to_laser = {
  range_min = 0.5;
  range_max = 130.0;
  angle_min = -math.pi;
  angle_max = math.pi;
  height_min = 1.5;
  height_max = 2.0;
  num_ranges = 360;
  pointcloud_topic = "ouster/points";
  laser_topic = "velodyne_2dscan_high_beams";
  source_frame = "os_sensor";
  target_frame = "base_link";
}
