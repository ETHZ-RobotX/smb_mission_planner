#!/usr/bin/env python2

import math
import rospy
import oyaml as yaml
import tf
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
import mission_plan


class MissionPlanner():
    def __init__(self, yaml_file_path, waypoint_topic_name, base_pose_topic_name):
        # Read missions data.
        self.yaml_file_path = yaml_file_path
        self.readMissionsData()

        # Set topic names to be easily used in mission definitions.
        global waypoint_topic_name_global
        global base_pose_topic_name_global
        waypoint_topic_name_global = waypoint_topic_name
        base_pose_topic_name_global = base_pose_topic_name

        self.main()

    def readMissionsData(self):
        with open(self.yaml_file_path, 'r') as file:
            self.missions_data = yaml.load(file, Loader=yaml.FullLoader)

    def main(self):
        rospy.init_node('mission_planner_node')
        rospy.loginfo("Mission planner started.")

        # Setup state machine.
        state_machine = mission_plan.createMissionPlan(self.missions_data)

        # Create and start the introspection server.
        introspection_server = smach_ros.IntrospectionServer('mission_planner_introspection_server', state_machine, '/mission_planner')
        introspection_server.start()

        # Execute state machine.
        outcome = state_machine.execute()
        rospy.loginfo("Mission plan terminated with outcome '" + outcome + "'.")

        # Wait for ctrl-c to stop the application
        introspection_server.stop()


class DefaultMission(smach.State):
    def __init__(self, mission_data):
        smach.State.__init__(self, outcomes=['Completed', 'Aborted', 'Next Waypoint'])
        self.mission_data = mission_data
        self.waypoint_idx = 0

        self.waypoint_pose_publisher = rospy.Publisher(waypoint_topic_name_global, PoseStamped, queue_size=1)
        self.base_pose_subscriber = rospy.Subscriber(base_pose_topic_name_global, PoseStamped, self.basePoseCallback)
        while self.waypoint_pose_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscriber to connect to '" +
                          waypoint_topic_name_global + "'.")
            rospy.sleep(1)
        while self.base_pose_subscriber.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for publisher to connect to '" +
                          base_pose_topic_name_global + "'.")
            rospy.sleep(1)

        self.countdown_s = 60
        self.countdown_decrement_s = 1
        self.distance_to_waypoint_tolerance_m = 0.3
        self.angle_to_waypoint_tolerance_rad = 0.7

    def execute(self, userdata):
        if(self.waypoint_idx >= len(self.mission_data.keys())):
            rospy.loginfo("No more waypoints left in current mission.")
            self.waypoint_idx = 0
            return 'Completed'

        current_waypoint_name = self.mission_data.keys()[self.waypoint_idx]
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

    def basePoseCallback(self, pose_stamped_msg):
        rospy.loginfo_once("Estimated base pose received from now on.")

        x_m = pose_stamped_msg.pose.position.x
        y_m = pose_stamped_msg.pose.position.y
        quaternion = pose_stamped_msg.pose.orientation
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

            return distance_to_waypoint_satisfied and angle_to_waypoint_satisfied
        except:
            rospy.logwarn("No estimated base pose received yet.")
            return False;
