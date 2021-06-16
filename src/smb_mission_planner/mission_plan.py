#!/usr/bin/env python

import smach
import rospy
import tf
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class MissionPlan():
    def __init__(self, missions_data, topic_names):
        self.missions_data = missions_data
        self.topic_names = topic_names


    def createStateMachine(self):
        state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
        with state_machine:
            smach.StateMachine.add('Mission 1', DefaultMission(self.missions_data['test_mission'], self.topic_names),
            transitions={'Completed':'Success', 'Aborted':'Failure', 'Next Waypoint':'Mission 1'})

        return state_machine


class DefaultMission(smach.State):
    def __init__(self, mission_data, topic_names):
        smach.State.__init__(self, outcomes=['Completed', 'Aborted', 'Next Waypoint'])
        self.mission_data = mission_data
        self.waypoint_idx = 0
        self.topic_names = topic_names

        self.waypoint_pose_publisher = rospy.Publisher(topic_names['waypoint'], PoseStamped, queue_size=1)
        self.base_pose_subscriber = rospy.Subscriber(topic_names['base_pose'], Odometry, self.basePoseCallback)
        while self.waypoint_pose_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once("Waiting for subscriber to connect to '" +
                          topic_names['waypoint'] + "'.")
            rospy.sleep(1)
        while self.base_pose_subscriber.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once("Waiting for publisher to connect to '" +
                          topic_names['base_pose'] + "'.")
            rospy.sleep(1)

        self.countdown_s = 60
        self.countdown_decrement_s = 1
        self.distance_to_waypoint_tolerance_m = 0.6
        self.angle_to_waypoint_tolerance_rad = 0.7

    def execute(self, userdata):
        if(self.waypoint_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        current_waypoint_name = list(self.mission_data.keys())[self.waypoint_idx]
        current_waypoint = self.mission_data[current_waypoint_name]

        self.setWaypoint(current_waypoint['x_m'], current_waypoint['y_m'], current_waypoint['yaw_rad'])
        rospy.loginfo("Waypoint set: '" + current_waypoint_name + "'.")

        countdown_s = self.countdown_s
        while countdown_s and not rospy.is_shutdown():
            if(self.reachedWaypointWithTolerance()):
                rospy.loginfo("Waypoint '" + current_waypoint_name + "' reached before countdown ended. Loading next waypoint...")
                self.waypoint_idx += 1
                return 'Next Waypoint'
            else:
                rospy.loginfo(str(countdown_s) + "s left until skipping waypoint '" + current_waypoint_name + "'.")
                rospy.sleep(self.countdown_decrement_s)
            countdown_s -= self.countdown_decrement_s
        rospy.logwarn("Countdown ended without reaching waypoint '" + current_waypoint_name + "'.")
        if(self.waypoint_idx == 0):
            rospy.logwarn("Starting waypoint of mission unreachable. Aborting current mission.")
            self.waypoint_idx = 0.
            return 'Aborted'
        else:
            rospy.logwarn("Skipping waypoint '" + current_waypoint_name + "'.")
            self.waypoint_idx += 1
            return 'Next Waypoint'

    def setWaypoint(self, x_m, y_m, yaw_rad):
        quaternion = tf.transformations.quaternion_from_euler(0., 0., yaw_rad)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.seq = 0
        pose_stamped_msg.header.stamp.secs = rospy.get_rostime().secs
        pose_stamped_msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        pose_stamped_msg.header.frame_id = "tracking_camera_odom"
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

    def basePoseCallback(self, Odometry_msg):
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = Odometry_msg.pose.pose.position.x
        y_m = Odometry_msg.pose.pose.position.y
        quaternion = Odometry_msg.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(explicit_quat)

        self.estimated_x_m = x_m
        self.estimated_y_m = y_m
        self.estimated_yaw_rad = yaw_rad

    def reachedWaypointWithTolerance(self):
        try:
            distance_to_waypoint = math.sqrt(pow(self.waypoint_x_m - self.estimated_x_m, 2) + pow(self.waypoint_y_m - self.estimated_y_m, 2))
            angle_to_waypoint = abs(self.waypoint_yaw_rad - self.estimated_yaw_rad)
            distance_to_waypoint_satisfied = (distance_to_waypoint <= self.distance_to_waypoint_tolerance_m)
            angle_to_waypoint_satisfied = (angle_to_waypoint <= self.angle_to_waypoint_tolerance_rad)
            # rospy.loginfo(distance_to_waypoint)
            return (distance_to_waypoint_satisfied and angle_to_waypoint_satisfied)
        except:
            rospy.logwarn("No estimated base pose received yet.")
            return False;
