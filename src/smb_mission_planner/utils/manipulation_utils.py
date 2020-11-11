#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import PyKDL


def frame_from_pose(pose):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                          pose.pose.orientation.z, pose.pose.orientation.w),
                       PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))


def pose_from_frame_and_header(frame, header):
    res = PoseStamped()
    res.pose.position.x = frame[(0, 3)]
    res.pose.position.y = frame[(1, 3)]
    res.pose.position.z = frame[(2, 3)]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z,
     res.pose.orientation.w) = frame.M.GetQuaternion()
    res.header = header
    return res


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
    iz = np.array([-yn, xn, 0.0]) / ((xn**2 + yn**2)**0.5)
    iy = np.array([0.0, 0.0, 1.0])

    R = np.ndarray(shape=(3, 3), dtype=float)
    R[:, 0] = -iy
    R[:, 1] = -iz
    R[:, 2] = -ix

#    R[:, 0] = ix
#    R[:, 1] = iy
#    R[:, 2] = iz

    # Generate just a box of size 0.5 py 0.5
    points = np.ndarray(shape=(3, 4))
    points[:, 0] = np.array([0.25, 0.25, 0.0])
    points[:, 1] = np.array([-0.25, 0.25, 0.0])
    points[:, 2] = np.array([-0.25, -0.25, 0.0])
    points[:, 3] = np.array([0.25, -0.25, 0.0])

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


def build_grinding_path_from_world_path(grinding_path, end_effector_pose_in_world_frame, distance_into_wall):
    assert isinstance(grinding_path, Path)
    assert isinstance(end_effector_pose_in_world_frame, PoseStamped)
    assert end_effector_pose_in_world_frame.header.frame_id == 'world'

    return_path = Path()
    return_path.header = grinding_path.header

    return_path.poses.append(end_effector_pose_in_world_frame)

    first_pose = frame_from_pose(grinding_path.poses[0])
    distance_vector_in_world_frame = first_pose.M*PyKDL.Vector(0, 0, distance_into_wall)

    initial_path_position = frame_from_pose(end_effector_pose_in_world_frame).p + distance_vector_in_world_frame

    translation_vector = initial_path_position - first_pose.p

    return_path.header.stamp = rospy.Time(0)


    last_translated_point = None

    for pose in grinding_path.poses:
        pose_as_frame = frame_from_pose(pose)
        translated_point = translation_vector + pose_as_frame.p
        translated_frame = PyKDL.Frame(pose_as_frame.M, translated_point)
        return_path.poses.append(pose_from_frame_and_header(translated_frame, pose.header))
        if last_translated_point is None:
            return_path.poses[-1].header.stamp = rospy.Time(0.1)
        else:
            difference_vector =  translated_point - last_translated_point
            velocity = 0.1
            travel_time = difference_vector.Norm()/velocity

            return_path.poses[-1].header.stamp = return_path.poses[-2].header.stamp + rospy.Duration(travel_time)

        last_translated_point = translated_point

    return return_path


if __name__ == "__main__":
    try:
        rospy.init_node('goal_from_plane', anonymous=True)

        path_pub = rospy.Publisher('/demo_path', Path, queue_size=10, latch=True)
        path_pub2 = rospy.Publisher('/demo_path2', Path, queue_size=10, latch=True)
        while not path_pub.get_num_connections():
            rospy.sleep(1.0)

        path_msg = generate_wall_path(offset=[2.0, 2.0, 0.5], xn=-1.0, yn=-1.0)

        path_pub.publish(path_msg)

        end_effector_pose_in_world_frame = PoseStamped()
        end_effector_pose_in_world_frame.header.frame_id = 'world'
        end_effector_pose_in_world_frame.pose.orientation.w = 1
        modified_path_msg = build_grinding_path_from_world_path(path_msg, end_effector_pose_in_world_frame, 0.02)
        path_pub2.publish(modified_path_msg)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
