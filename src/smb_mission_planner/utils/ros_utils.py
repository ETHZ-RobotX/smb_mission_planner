import os
import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse


def get_param_safe(param_name):
    """
    Return the parameter if it exists, otherwise it raises an exception
    :param param_name:
    :return:
    """
    try:
        return rospy.get_param(param_name)
    except KeyError as exc:
        rospy.logerr(exc)
        raise NameError("Failed to parse parameter {}".format(param_name))


def switch_ros_controller(controller_name, manager_namespace='', whitelist=[]):
    list_controller_service_name = os.path.join(manager_namespace, "controller_manager", "list_controllers")
    try:
        rospy.wait_for_service(list_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(list_controller_service_name))
        return False

    list_service_client = rospy.ServiceProxy(list_controller_service_name, ListControllers)
    req = ListControllersRequest()
    res = list_service_client.call(req)
    res = ListControllersResponse()

    # stop all controllers running which are not in the whitelist and not the controller we want to switch to
    whitelist.append(controller_name)
    whitelist_set = set(whitelist)
    controller_set = set(res.controller)
    controller_stop_list = list(controller_set - whitelist_set)
    controller_start_list = [controller_name]

    switch_controller_service_name = os.path.join(manager_namespace, "controller_manager", "switch_controllers")
    try:
        rospy.wait_for_service(list_controller_service_name, timeout=2.0)
    except rospy.ROSException as exc:
        rospy.logerr("Failed to call {}".format(switch_controller_service_name))
        return False

    switch_controller_client = rospy.ServiceProxy(switch_controller_service_name, SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers = controller_start_list
    req.stop_controllers = controller_stop_list
    req.timeout = 5.0
    req.start_asap = True
    req.strictness = req.STRICT

    res = switch_controller_client.call(req)
    if not res.ok:
        rospy.logerr("Failed to switch ros controller")
        return False

    return True
