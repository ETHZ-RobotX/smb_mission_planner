#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Path
from smb_mission_planner.utils.manipulation_utils import generate_wall_path


if __name__ == "__main__":
    try:
        rospy.init_node('path_generator', anonymous=True)
        path_topic_name = rospy.get_param("~path_topic_name")
        x0 = rospy.get_param("~x0", 0.0)
        y0 = rospy.get_param("~y0", 0.0)
        z0 = rospy.get_param("~z0", 0.0)
        yaw = rospy.get_param("~yaw_deg", 0.0)
        yaw = math.pi * yaw / 180.0

        path_pub = rospy.Publisher(path_topic_name, Path, queue_size=10, )
        while not path_pub.get_num_connections():
            rospy.sleep(1.0)

        path_msg = generate_wall_path(offset=[x0, y0, z0], xn=math.cos(yaw), yn=math.sin(yaw))
        while not rospy.is_shutdown():
            path_pub.publish(path_msg)
            rospy.sleep(2.0)

    except rospy.ROSInterruptException:
        pass
