#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse

from cpt_pointlaser_msgs.srv import HighAccuracyLocalizationResponse, HighAccuracyLocalization
from geometry_msgs.msg import PoseStamped, Pose
from rocoma_msgs.srv import SwitchControllerResponse, SwitchController
from mabi_speedy12_msgs.srv import DesiredWrenchCurrentEEFrameReference, DesiredWrenchCurrentEEFrameReferenceResponse


def planner_callback(msg):
    """ Receive the waypoint and after some time publish it as the new base pose"""
    rospy.loginfo("Planner received new goal")
    rospy.sleep(1.0)
    base_pose_publisher.publish(msg)  # the goal is immediately reached


def controller_callback(msg):
    """ Receive the waypoint and after some time publish it as the new base pose"""
    rospy.loginfo("Controller received new goal in frame {}".format(msg.header.frame_id))
    rospy.loginfo("Translation: {}, {}, {}".format(msg.pose.position.x,
                                                   msg.pose.position.y,
                                                   msg.pose.position.z))
    rospy.loginfo("Rotation: {}, {}, {}, {}".format(msg.pose.orientation.x,
                                                    msg.pose.orientation.y,
                                                    msg.pose.orientation.z,
                                                    msg.pose.orientation.w))
    rospy.sleep(1.0)


def switch_roco_controller_service(req):
    rospy.sleep(1.0)
    rospy.loginfo("Switching to controller: " + str(req.name))
    res = SwitchControllerResponse()
    res.status = res.STATUS_SWITCHED
    return res


def hal_data_callback(_):
    rospy.loginfo("Received a hal data collection request. Sleeping 1.0 sec and returning")
    rospy.sleep(1.0)
    return EmptyResponse()


def hal_optimization_callback(_):
    rospy.loginfo("Received a hal data collection request. Sleeping 1.0 sec and then sending the update")
    rospy.sleep(1.0)
    hal_response = HighAccuracyLocalizationResponse()
    hal_response.corrected_base_pose_in_world = Pose()
    return hal_response


def set_wrench_callback(req):
    rospy.loginfo("Requested to set new wrench: {}.\nSleeping for 1.0 sec and returning".format(req))
    rospy.sleep(1.0)
    res = DesiredWrenchCurrentEEFrameReferenceResponse()
    res.success = True
    res.message = "Reference set has been set."
    return res


if __name__ == "__main__":
    rospy.init_node("hilti_mock_servers")

    path_topic_name = rospy.get_param("~path_topic_name")
    nav_goal_topic = rospy.get_param("~nav_goal_topic")
    base_odom_topic = rospy.get_param("~base_odom_topic")

    ee_goal_topic = rospy.get_param("~ee_goal_topic")
    controller_manager_ns = rospy.get_param("~controller_manager_namespace", "/smb_highlevel_controller")

    hal_optimization_service_name = rospy.get_param("~hal_optimization_service_name")
    hal_data_collection_service_name = rospy.get_param("~hal_data_collection_service_name")
    wrench_service_name = rospy.get_param("~wrench_service_name")

    nav_goal_subscriber = rospy.Subscriber(nav_goal_topic, PoseStamped, planner_callback, queue_size=10)
    ee_goal_subscriber = rospy.Subscriber(ee_goal_topic, PoseStamped, controller_callback, queue_size=10)

    base_pose_publisher = rospy.Publisher(base_odom_topic, PoseStamped, queue_size=10)

    roco_service = rospy.Service(controller_manager_ns + "/controller_manager/switch_controller",
                                 SwitchController,
                                 switch_roco_controller_service)

    hal_data_service = rospy.Service(hal_data_collection_service_name, Empty, hal_data_callback)
    hal_optm_service = rospy.Service(hal_optimization_service_name, HighAccuracyLocalization, hal_optimization_callback)
    wrench_service = rospy.Service(wrench_service_name, DesiredWrenchCurrentEEFrameReference, set_wrench_callback)

    # Spin
    rospy.spin()
