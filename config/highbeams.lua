pointcloud_to_laser = {
  range_min = 0.2;
  range_max = 130.0;
  angle_min = -math.pi;
  angle_max = math.pi;
  height_min = 0.75;
  height_max = 2.0;
  num_ranges = 1000;
  laser_topic = "velodyne_2dscan_highbeams";
  pointcloud_topic = "velodyne_points";
}
