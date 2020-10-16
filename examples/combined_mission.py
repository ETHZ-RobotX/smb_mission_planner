import rospy
import smach
import smach_ros
from smb_mission_planner.navigation_states import WaypointNavigation
from smb_mission_planner.manipulation_states import MoveItHome, JointsConfigurationVisitor
from smb_mission_planner.detection_states import ObjectDetectionWithService

from smb_mission_planner.utils import ros_utils
"""
Example script of a mission which combines navigation and manipulator controller through the moveit interface
In this simple mission the robot reaches a predefined configuration, navigates to a predefined goal, 
scans the environment and upon successful detection 
"""

rospy.init_node('combined_mission_node')

# File of prerecorded waypoint-base navigation missions
navigation_file = ros_utils.get_param_safe("mission_file")

# Topic listening for the global planner goals
move_base_topic = ros_utils.get_param_safe("move_base_topic")

# Topic where the base odometry is published. Used as reference for navigation
odometry_topic = ros_utils.get_param_safe("odometry_topic")

# Name of the arm group that moveit will plan trajectories for
arm_moveit_group = ros_utils.get_param_safe("arm_moveit_group")

# Parse the missions data
missions_data = WaypointNavigation.read_missions_data(navigation_file)


state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
with state_machine:
    smach.StateMachine.add('HOME_ROBOT', MoveItHome(ns="HOME_ROBOT"),
                           transitions={'Completed': 'REACH_DETECTION_HOTSPOT',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('REACH_DETECTION_HOTSPOT', WaypointNavigation(missions_data['detection_mission'],
                                                                         waypoint_pose_topic=move_base_topic,
                                                                         base_pose_topic=odometry_topic,
                                                                         ns="REACH_DETECTION_HOTSPOT"),
                           transitions={'Completed': 'DETECT',
                                        'Aborted': 'Failure',
                                        'Next Waypoint': 'REACH_DETECTION_HOTSPOT'})

    smach.StateMachine.add('DETECT', ObjectDetectionWithService(max_num_failure=2, ns='DETECT'),
                           transitions={'Completed': 'Success',
                                        'Failure': 'Failure',
                                        'Retry': 'Mission 1'})

    smach.StateMachine.add('NEW_VIEWPOINT', JointsConfigurationVisitor(ns='NEW_VIEWPOINT'),
                           transitions={'Completed': 'DETECT',
                                        'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
