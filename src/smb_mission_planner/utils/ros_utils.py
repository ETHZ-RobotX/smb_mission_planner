import rospy


def get_param_safe(param_name):
    if not rospy.search_param(param_name):
        raise NameError("No param {} found".format(param_name))
    else:
        return rospy.get_param(param_name)
