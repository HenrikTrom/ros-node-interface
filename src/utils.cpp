#include <ros-node-interface/utils.hpp>

namespace ros_node_interface{
    
bool topic_exists(std::string topic_name){

    // Get the list of all topics
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    // Check if the desired topic exists
    bool tpc_exists = false;
    for (const auto &topic_info : topics)
    {
        if (topic_info.name == topic_name)
        {
            tpc_exists = true;
            break;
        }
    }
    return tpc_exists;
}

} //namespace ros_node_interface