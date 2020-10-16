#!/usr/bin/env python2

import tf
import yaml
import math
import rospy
from geometry_msgs.msg import PoseStamped

from smb_mission_planner.base_state_ros import BaseStateRos
"""
Here define all the navigation related states
"""


class WaypointNavigation(BaseStateRos):
    """
    In this state the robot navigates through a sequence of waypoint which are sent to the
    global planner once the previous has been reached within a certain tolerance
    """
    def __init__(self, mission, waypoint_pose_topic, base_pose_topic, ns=""):
        BaseStateRos.__init__(self, outcomes=['Completed', 'Aborted', 'Next Waypoint'], ns="")
        self.mission_data = mission
        self.waypoint_idx = 0

        self.waypoint_pose_publisher = rospy.Publisher(waypoint_pose_topic, PoseStamped, queue_size=10)
        self.base_pose_subscriber = rospy.Subscriber(base_pose_topic, PoseStamped, self.base_pose_callback)

        self.countdown_s = 60
        self.countdown_decrement_s = 1
        self.distance_to_waypoint_tolerance_m = 0.3
        self.angle_to_waypoint_tolerance_rad = 0.7

        self.waypoint_x_m = 0.
        self.waypoint_y_m = 0.
        self.waypoint_yaw_rad = 0.

        self.estimated_x_m = 0.
        self.estimated_y_m = 0.
        self.estimated_yaw_rad = 0.

    @staticmethod
    def read_missions_data(mission_file):
        """
        Reads the mission data and return the corresponding dictionary
        :param mission_file:
        :return:
        """
        assert mission_file.endswith(".yaml")
        with open(mission_file, 'r') as stream:
            return yaml.load(stream)

    def execute(self, userdata):
        if self.waypoint_idx >= len(self.mission_data.keys()):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        current_waypoint_name = self.mission_data.keys()[self.waypoint_idx]
        current_waypoint = self.mission_data[current_waypoint_name]

        self.set_waypoint(current_waypoint['x_m'], current_waypoint['y_m'], current_waypoint['yaw_rad'])
        rospy.loginfo("Waypoint set: '" + current_waypoint_name + "'.")

        countdown_s = self.countdown_s
        while countdown_s and not rospy.is_shutdown():
            if self.reached_waypoint_with_tolerance():
                rospy.loginfo("Waypoint '" + current_waypoint_name +
                              "' reached before countdown ended. Loading next waypoint...")
                self.waypoint_idx += 1
                return 'Next Waypoint'
            else:
                rospy.loginfo(str(countdown_s) + "s left until skipping waypoint '" + current_waypoint_name + "'.")
                rospy.sleep(self.countdown_decrement_s)
            countdown_s -= self.countdown_decrement_s
        rospy.logwarn("Countdown ended without reaching waypoint '" + current_waypoint_name + "'.")
        if self.waypoint_idx == 0:
            rospy.logwarn("Starting waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'
        else:
            rospy.logwarn("Skipping waypoint '" + current_waypoint_name + "'.")
            self.waypoint_idx += 1
            return 'Next Waypoint'

    def set_waypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw_rad)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.seq = 0
        pose_stamped_msg.header.stamp.secs = rospy.get_rostime().secs
        pose_stamped_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        pose_stamped_msg.header.frame_id = "world"
        pose_stamped_msg.pose.position.x = x_m
        pose_stamped_msg.pose.position.y = y_m
        pose_stamped_msg.pose.position.z = 0.
        pose_stamped_msg.pose.orientation.x = quaternion[0]
        pose_stamped_msg.pose.orientation.y = quaternion[1]
        pose_stamped_msg.pose.orientation.z = quaternion[2]
        pose_stamped_msg.pose.orientation.w = quaternion[3]
        self.waypoint_pose_publisher.publish(pose_stamped_msg)

        self.waypoint_x_m = x_m
        self.waypoint_y_m = y_m
        self.waypoint_yaw_rad = yaw_rad

    def base_pose_callback(self, pose_stamped_msg):
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        self.estimated_x_m = x_m
        self.estimated_y_m = y_m
        self.estimated_yaw_rad = yaw_rad

    def reached_waypoint_with_tolerance(self):
        try:
            distance_to_waypoint = math.sqrt(pow(self.waypoint_x_m - self.estimated_x_m, 2) +
                                             pow(self.waypoint_y_m - self.estimated_y_m, 2))
            angle_to_waypoint = abs(self.waypoint_yaw_rad - self.estimated_yaw_rad)
            distance_to_waypoint_satisfied = (distance_to_waypoint <= self.distance_to_waypoint_tolerance_m)
            angle_to_waypoint_satisfied = (angle_to_waypoint <= self.angle_to_waypoint_tolerance_rad)

            return distance_to_waypoint_satisfied and angle_to_waypoint_satisfied
        except:
            rospy.logwarn("No estimated base pose received yet.")
            return False

