#!/usr/bin/env python
import numpy as np
import rospy
import time
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from amrl_msgs.msg import NavStatusMsg
import argparse


class WaypointNav:
    WAYPOINTS = {
        "ahg2": [
            (10.4, 74.4, 0.0, 1.0),
            (4.1, 70.8, 0.0, 1.0),
            (12.5, 69.6, 0.0, 1.0),
        ],
        "courtyard": [
            (-10.1, 16.7, 0.0, 1.0),
            (-1.7, 11.2, 0.0, 1.0),
            (11.1, 17.1, 0.0, 1.0),
            (-2.1, 23.1, 0.0, 1.0),
        ],
        "tourguide": [
            (-10.031, 16.859, 0.0, 1.0),  # ahg
            (61.820, -84.904, 0.135, 0.991),  # nhb
            (80.116, -227.031, 0.707, 0.707),  # gdc
            (61.820, -84.904, 0.135, 0.991),  # nhb
        ],
    }

    def __init__(self, stop_time, map):
        self.stop_time = stop_time
        self.map = map
        self.traj = self.WAYPOINTS[map]
        self.next_goal = 0
        self.stop_detected_first_time = -1

        rospy.Subscriber('/navigation_goal_status', NavStatusMsg, self.nav_callback, queue_size=1)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def nav_callback(self, msg):
        pub_msg = PoseStamped()
        if msg.status == 0:  # stopped
            if self.stop_detected_first_time == -1:
                self.stop_detected_first_time = rospy.get_rostime().secs
            else:
                if rospy.get_rostime().secs - self.stop_detected_first_time > self.stop_time:
                    self.stop_detected_first_time = -1
                    # publish next goal
                    pub_msg.pose.position.x = self.traj[self.next_goal][0]
                    pub_msg.pose.position.y = self.traj[self.next_goal][1]
                    pub_msg.pose.orientation.z = self.traj[self.next_goal][2]
                    pub_msg.pose.orientation.w = self.traj[self.next_goal][3]
                    self.pub.publish(pub_msg)
                    time.sleep(4)
                    self.next_goal += 1
                    self.next_goal %= len(self.traj)
        else:
            self.stop_detected_first_time = -1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop_time", type=int, required=True, help="Time to stop at each waypoint")
    parser.add_argument("--map", type=str, required=True, help="Map to use: ahg2/courtyard/tourguide")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('waypoint_nav', anonymous=False)
    obj = WaypointNav(stop_time=args.stop_time, map=args.map)
    time.sleep(1)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS waypoint nav node")
