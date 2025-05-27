#pragma once

#include <iostream>
#include <string>
#include "ros/ros.h"

namespace ros_node_interface{

/**
 * Checks if parameter is availiable and gets it
 *
 * @param [in] nh Nodehandle 
 * @param [in] name Name of the parameter
 * @param [out] parameter return parameter
 * @return true if parameter exists, false otherwise
 */
template<typename paramtype>
bool check_get_param(ros::NodeHandle& nh, const std::string& name, paramtype& parameter){
    std::string error_msg;
    if (!nh.hasParam(name))
    {
        error_msg = "Parameter "+name+" does not exist";
        ROS_ERROR_STREAM(error_msg);
        return false;
    }
    nh.getParam(name, parameter);
    return true;
};

/**
 * Checks if rostopic exists
 *
 * @param [in] topic_name Name of the topic
 * \return true if topic exist, false otherwise.
 */
bool topic_exists(std::string topic_name);

} // namespace ros_node_interface