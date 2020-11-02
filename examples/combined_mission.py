#!/usr/bin/env python2

import rospy
import smach
import smach_ros
from smb_mission_planner.navigation_states import WaypointNavigation
from smb_mission_planner.manipulation_states import *
from smb_mission_planner.detection_states import ObjectDetectionWithService

from smb_mission_planner.utils import ros_utils
"""
Example script of a mission which combines navigation and manipulator controller through the moveit interface
In this simple mission the robot reaches a predefined configuration, navigates to a predefined goal, 
scans the environment and upon successful detection 
"""

rospy.init_node('combined_mission_node')

# File of prerecorded waypoint-base navigation missions
navigation_file = ros_utils.get_param_safe("~mission_file")

# Topic listening for the global planner goals
move_base_topic = ros_utils.get_param_safe("~move_base_topic")

# Topic where the base odometry is published. Used as reference for navigation
odometry_topic = ros_utils.get_param_safe("~odometry_topic")

# Parse the missions data
missions_data = WaypointNavigation.read_missions_data(navigation_file)
assert missions_data.has_key('detection')


# Build the state machine
state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
with state_machine:
    smach.StateMachine.add('HOME_ROBOT', MoveItHome(ns="home_robot"),
                           transitions={'Completed': 'REACH_DETECTION_HOTSPOT',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('REACH_DETECTION_HOTSPOT', WaypointNavigation(missions_data['detection'],
                                                                         waypoint_pose_topic=move_base_topic,
                                                                         base_pose_topic=odometry_topic,
                                                                         ns="reach_detection_hotspot"),
                           transitions={'Completed': 'DETECT',
                                        'Aborted': 'Failure',
                                        'Next Waypoint': 'REACH_DETECTION_HOTSPOT'})

    smach.StateMachine.add('DETECT', ObjectDetectionWithService(max_num_failure=3, ns='detect'),
                           transitions={'Completed': 'Success',
                                        'Failure': 'Failure',
                                        'Retry': 'NEW_VIEWPOINT'})

    smach.StateMachine.add('NEW_VIEWPOINT', JointsConfigurationVisitor(ns='new_viewpoint'),
                           transitions={'Completed': 'DETECT',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
                           transitions={'Completed': 'REACH_GRASP',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('REACH_GRASP', MoveItPoseReaching(ns='reach_grasp'),
                           transitions={'Completed': 'CLOSE_GRIPPER',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('CLOSE_GRIPPER', JointsConfigurationVisitor(ns='close_gripper'),
                           transitions={'Completed': 'EXECUTE_EE_TRAJECTORY',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('EXECUTE_EE_TRAJECTORY', RosControlPoseReaching(ns='execute_ee_trajectory'),
                           transitions={'Completed': 'Success',
                                        'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
