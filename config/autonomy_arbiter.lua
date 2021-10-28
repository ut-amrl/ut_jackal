AutonomyArbiterParameters = {
  src_topic = "navigation/cmd_vel",
  dest_topic = "cmd_vel",
  status_topic = "autonomy_arbiter/enabled",
  joystick_topic = "bluetooth_teleop/joy",
  start_btn_idx = 0,
  recording_topics = {
    "/status",
    "/imu/data",
    "/bluetooth_teleop/joy",
    "/left/image_color/compressed",
    "/right/image_color/compressed",
    "/velodyne_2dscan_high_beams",
    "/jackal_velocity_controller/odom",
    "/velodyne_2dscan",
    "/odometry/filtered",
    "/tf",
    "/localization",
    "/move_base_simple/goal",
    "/navigation/cmd_vel",
    "/set_nav_target",
    "/set_pose",
    "/camera/rgb/image_raw/compressed",
    "/camera/depth/image_raw/compressed",
    "/velodyne_points",
    "/navsat/nmea_sentence",
    "/imu/data_raw",
    "/visualization"
  },
  record_directory = "/data/"
}

