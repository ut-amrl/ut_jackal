AutonomyArbiterParameters = {
  src_topic = "navigation/cmd_vel",
  dest_topic = "cmd_vel",
  status_topic = "autonomy_arbiter/enabled",
  joystick_topic = "bluetooth_teleop/joy",
  start_btn_idx = 0,
  joystick_stamp_topic = "/autonomy_arbiter/joystick_stamp",
  joystick_feedback_topic ="/joy_teleop/joy/set_feedback",
  recording_topics = {
    -- jackal data
    "/status",
    "/imu/data_raw",
    "/jackal_velocity_controller/odom",
    "/bluetooth_teleop/joy",
    "/navsat/nmea_sentence",
    "/imu/data",
    "/odometry/filtered",
    -- "/tf",

    -- velodyne
    -- "/velodyne_2dscan_high_beams",
    -- "/velodyne_2dscan",
    -- "/velodyne_points",
    -- "/velodyne_packets",

    -- ouster
    -- "/ouster/points",
    "/ouster/lidar_packets",

    -- zed -- need to verify these
    "/zed2i/zed_node/left/camera_info",
    "/zed2i/zed_node/right/camera_info",
    -- "/zed2i/zed_node/depth/camera_info",
    "/zed2i/zed_node/left/image_rect_color/compressed",
    "/zed2i/zed_node/right/image_rect_color/compressed",
    -- "/zed2i/zed_node/depth/depth_registered/compressedDepth",
    -- "/zed2i/zed_node/odom",

    "/visualization",

    -- localization
    "/localization",
    "/set_pose",

    -- navigation
    "/carrot",
    "/move_base_simple/goal",
    "/navigation/cmd_vel",
    "/set_nav_target",
    "/autonomy_arbiter/enabled",

    -- Cameras
    -- "/left/image_color/compressed",
    -- "/right/image_color/compressed",
    -- "/camera/rgb/image_raw/compressed",
    -- "/camera/depth/image_raw/compressed",

    -- Metadata
    "/error_report",
    "/autonomy_arbiter/joystick_stamp"
  },
  record_directory = "/data/"
}

