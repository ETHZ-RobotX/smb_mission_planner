#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from smb_mission_planner.utils.manipulation_utils import generate_wall_path


if __name__ == "__main__":
    try:
        rospy.init_node('path_generator', anonymous=True)

        path_pub = rospy.Publisher('/demo_path', Path, queue_size=10, )
        while not path_pub.get_num_connections():
            rospy.sleep(1.0)

        path_msg = generate_wall_path(offset=[4.0, 4.0, 1.5], xn=-1.0, yn=-1.0)
        while not rospy.is_shutdown():
            path_pub.publish(path_msg)
            rospy.sleep(2.0)

    except rospy.ROSInterruptException:
        pass
