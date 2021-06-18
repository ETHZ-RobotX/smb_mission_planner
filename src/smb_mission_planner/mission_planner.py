#!/usr/bin/env python

import math
import rospy
import yaml
import tf
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import smb_mission_planner.mission_plan


class MissionPlanner():
    def __init__(self, yaml_file_path, waypoint_topic_name, base_pose_topic_name):
        # Read missions data.
        self.yaml_file_path = yaml_file_path
        self.readMissionsData()

        self.topic_names = {'waypoint': waypoint_topic_name,
        'base_pose': base_pose_topic_name}

        self.main()

    def readMissionsData(self):
        with open(self.yaml_file_path, 'r') as file:
            self.missions_data = yaml.load(file, Loader=yaml.FullLoader)

    def main(self):
        rospy.init_node('mission_planner_node')
        rospy.loginfo("Mission planner started.")

        # Setup state machine.
        mission_plan = smb_mission_planner.mission_plan.MissionPlan(self.missions_data, self.topic_names)
        state_machine = mission_plan.createStateMachine()

        # Create and start the introspection server.
        introspection_server = smach_ros.IntrospectionServer('mission_planner_introspection_server', state_machine, '/mission_planner')
        introspection_server.start()

        # Execute state machine.
        outcome = state_machine.execute()
        rospy.loginfo("Mission plan terminated with outcome '" + outcome + "'.")

        # Wait for ctrl-c to stop the application
        introspection_server.stop()