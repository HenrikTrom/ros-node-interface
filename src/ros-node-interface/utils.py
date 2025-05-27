import rospy

def check_get_param(param_name: str):
    """
    Checks if parameter exists and returns it
    """
    if not rospy.has_param(param_name):
        rospy.logerr(f'Parameter not found: {param_name}')
        raise RuntimeError(f'Parameter not found: {param_name}')
    return rospy.get_param(param_name)
