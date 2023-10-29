pointcloud_to_laser = {
  range_min = 0.1;
  range_max = 130.0;
  angle_min = (-math.pi * 3 / 4);
  angle_max = (math.pi * 3 / 4);
  height_min = 0.1;
  height_max = 1.0;
  num_ranges = 270;
  pointcloud_topic = "/ouster/points";
  laser_topic = "/velodyne_2dscan";
  source_frame = "os_sensor";
  target_frame = "base_link";
}
