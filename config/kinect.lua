serial = "";
costmap_topic = "kinect_costmap";
points_topic = "kinect_points";
rgb_image_topic = "camera/rgb/image_raw";
depth_image_topic = "camera/depth/image_raw";
rgb_image_frame = "kinect_color";
depth_image_frame = "kinect_depth";
scan_frame = "base_link";
scan_topic = "kinect_laserscan";
imu_topic = "kinect_imu";
imu_frame = "kinect_imu";
registered_rgbd = true;

rotation = {
  yaw = 0;
  pitch = -2.6;
  roll = 180;
};

translation = {
  x = -0.06;
  y = 0.05;
  z = .53;
};

skip_points = 10;
ground_dist_thresh = 0.04;
ground_angle_thresh = 5.0; -- Degrees.
camera_angle_thresh = 50.0;
min_dist_thresh = 0.05;
num_ranges = 180;
