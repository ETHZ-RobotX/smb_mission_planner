#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from smb_mission_planner.srv import DetectObject, DetectObjectResponse
from smb_mission_planner.utils import ros_utils

rospy.init_node("mock_servers")


# Navigation Servers
move_base_topic = ros_utils.get_param_safe("~move_base_topic")
odometry_topic = ros_utils.get_param_safe("~odometry_topic")
rospy.loginfo("Mock server listening on: {}".format(move_base_topic))


def planner_callback(msg):
    """ Receive the waypoint and after some time publish it as the new base pose"""
    rospy.loginfo("Planner received new waypoint")
    rospy.sleep(1.0)
    base_pose_publisher.publish(msg)


global_planner_subscriber = rospy.Subscriber(move_base_topic, PoseStamped, planner_callback, queue_size=10)
base_pose_publisher = rospy.Publisher(odometry_topic, PoseStamped, queue_size=10)


# Detection Servers
attempts = 0

def detection_callback(req):
    global attempts
    res = DetectObjectResponse()
    res.object_pose = PoseStamped()
    if attempts == 0:
        res.success = False
    else:
        res.success = True
    attempts += 1
    return res


rospy.Service("/detection_service", DetectObject, detection_callback)

# Spin
rospy.spin()
