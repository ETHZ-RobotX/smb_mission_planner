#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


def generate_wall_path(offset, xn, yn, frame_id="world"):
    """
    Generate a path on a surface defined by origin and normal
    :param offset: the offset of the path
    :param xn: the x component of the normal direction of the surface
    :param yn: the y component of the normal direction of the surface
    :param frame_id: the frame id for the path msg
    :return:
    """
    if not len(offset) == 3:
        rospy.logerr("Offset must have dimension 3. Got {}".format(len(offset)))

    ix = np.array([xn, yn, 0.0]) / ((xn**2 + yn**2)**0.5)
    iy = np.array([-yn, xn, 0.0]) / ((xn**2 + yn**2)**0.5)
    iz = np.array([0.0, 0.0, 1.0])

    R = np.ndarray(shape=(3, 3), dtype=float)
    R[:, 0] = ix
    R[:, 1] = iy
    R[:, 2] = iz

    # Generate just a box of size 0.5 py 0.5
    points = np.ndarray(shape=(3, 4))
    points[:, 0] = np.array([0.0, 0.25, 0.25])
    points[:, 1] = np.array([0.0, -0.25, 0.25])
    points[:, 2] = np.array([0.0, -0.25, -0.25])
    points[:, 3] = np.array([0.0, 0.25, -0.25])

    # Rotate plane
    points_rotated = R.dot(points)
    rot = Rotation.from_dcm(R)
    rot_quat = rot.as_quat()

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = rospy.get_rostime()
    for i in range(4):
        new_pose = PoseStamped()
        new_pose.pose.orientation.x = rot_quat[0]
        new_pose.pose.orientation.y = rot_quat[1]
        new_pose.pose.orientation.z = rot_quat[2]
        new_pose.pose.orientation.w = rot_quat[3]
        new_pose.pose.position.x = offset[0] + points_rotated[0, i]
        new_pose.pose.position.y = offset[1] + points_rotated[1, i]
        new_pose.pose.position.z = offset[2] + points_rotated[2, i]
        path.poses.append(new_pose)

    return path


if __name__ == "__main__":
    try:
        rospy.init_node('goal_from_plane', anonymous=True)

        path_pub = rospy.Publisher('/demo_path', Path, queue_size=10, )
        while not path_pub.get_num_connections():
            rospy.sleep(1.0)

        path_msg = generate_wall_path(offset=[4.0, 4.0, 1.5], xn=-1.0, yn=-1.0)
        path_pub.publish(path_msg)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
