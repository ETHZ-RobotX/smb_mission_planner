#!/usr/bin/env python2

import rospy
from smb_mission_planner.utils import NavigationMissionRecorder
from smb_mission_planner.utils.ros_utils import get_param_safe

rospy.init_node("navigation_mission_recorder")
file_path = get_param_safe("file_path")
waypoint_topic = get_param_safe("waypoint_topic")
base_pose_topic = get_param_safe("base_pose_topic")

recorder = NavigationMissionRecorder(file_path=file_path,
                                     waypoint_topic=waypoint_topic,
                                     base_pose_topic=base_pose_topic)
recorder.run()
