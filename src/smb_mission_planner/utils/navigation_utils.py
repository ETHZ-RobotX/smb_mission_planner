#!/usr/bin/env python
""" 
@file   goal_from_plane.py
@brief  ROS Node that generates a goal for move base from a path on a plane
@author Luca Bartolomei, V4RL
@date   04.11.2020
"""

import numpy as np
from numpy.linalg import svd
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def fit_plane(points):
    """
    p, n = planeFit(points)
    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fit an d-dimensional plane to the points.
    Return a point, p, on the plane (the point-cloud centroid),
    and the normal, n.
    """    
    # Collapse trialing dimensions
    points = np.reshape(points, (np.shape(points)[0], -1)) 
    assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
    ctr = points.mean(axis=1)
    x = points - ctr[:,np.newaxis]
    M = np.dot(x, x.T) # Could also use np.cov(x) here.
    return ctr, svd(M)[0][:,-1]


def nav_goal_from_path(path_msg, frame_id="world", distance_from_wall=3.0, normal_direction=1, do_plots=False):
    points = np.array([])
    for pose in path_msg.poses:
        position = np.array([pose.pose.position.x, 
                             pose.pose.position.y, 
                             pose.pose.position.z])
        if points.shape[0] == 0:
            points = position
        else:
            points = np.vstack((points, position))
    
    rospy.loginfo("This path has {} points".format(points.shape[0]))
    centroid, normal = fit_plane(points.T)
    
    # Compute the goal position: the goal is at a fixed distance from the 
    # centroid along the normal, with orientation such that we face the wall
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = frame_id
    goal_msg.header.stamp = rospy.Time.now()
    
    goal_msg.pose.position.x = normal_direction * distance_from_wall * normal[0] + centroid[0]
    goal_msg.pose.position.y = normal_direction * distance_from_wall * normal[1] + centroid[1]
    goal_msg.pose.position.z = normal_direction * distance_from_wall * normal[2] + centroid[2]
    
    # Since the path is in CAD world frame, we can project the normal on the 
    # (x,y) plane and compute the yaw from that.
    normal_xy = -normal_direction * normal[0:2] / np.linalg.norm(normal[0:2])
    yaw = math.atan2(normal_xy[1], normal_xy[0])
    rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False)
    rot_quat = rot.as_quat()
    goal_msg.pose.orientation.x = rot_quat[0]
    goal_msg.pose.orientation.y = rot_quat[1]
    goal_msg.pose.orientation.z = rot_quat[2]
    goal_msg.pose.orientation.w = rot_quat[3]

    # Inform user
    rospy.loginfo("Plane Centroid: {}".format(centroid))
    rospy.loginfo("Plane Normal  : {}".format(normal))
    rospy.loginfo("Yaw           : {}".format(yaw))
    rospy.loginfo("Quaternion    : {}".format(rot_quat))
    
    # Plotting
    if do_plots:
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2])
        ax.scatter(goal_msg.pose.position.x, goal_msg.pose.position.y,
                   goal_msg.pose.position.z)
        ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
        plt.show()

    return goal_msg


def test_callback(msg):
    goal = nav_goal_from_path(msg, "world", 3.0, 1, False)
    goal_pub.publish(goal)


if __name__ == '__main__':
    try:
        rospy.init_node('goal_from_plane', anonymous=True)
        
        # Subscribers and Publishers
        path_sub = rospy.Subscriber("input_path", Path, test_callback)
        goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10,)
        rospy.loginfo("Waiting for path message...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass