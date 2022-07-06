#!/usr/bin/env python

import rospy
import math
from math import fabs
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped


yaw_z = 0
pub = rospy.Publisher('/waypoint_yaw', Float64)


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return yaw_z # in radians
        
        
def transform_callback(pose_stamped):
    global yaw_z
    yaw_z = euler_from_quaternion(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)
    pub.publish(Float64(yaw_z))


def read_goal():
    rospy.init_node('waypoint_yaw', anonymous=True)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, transform_callback)
    # Initial movement.
    pub.publish(Float64(yaw_z))
    rospy.spin()


if __name__ == '__main__':
    try:
        read_goal()
    except rospy.ROSInterruptException:
        pass