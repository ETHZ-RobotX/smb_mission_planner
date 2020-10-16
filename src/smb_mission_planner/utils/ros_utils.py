import rospy


def get_param_safe(param_name):
    try:
        return rospy.get_param(param_name)
    except KeyError as exc:
        rospy.logerr(exc)
        raise NameError("Failed to parse parameter {}".format(param_name))
