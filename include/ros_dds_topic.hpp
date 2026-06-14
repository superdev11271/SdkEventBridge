#ifndef ROS_DDS_TOPIC_HPP
#define ROS_DDS_TOPIC_HPP

#include <string>

namespace sdk_event_bridge
{

// ROS 2 (rmw_cyclonedds) maps "/cmd_vel_origin" to the DDS topic "rt/cmd_vel_origin".
// Unitree ChannelPublisher uses the topic string verbatim, so convert here.
inline std::string ToRosDdsTopicName(const std::string& topicName)
{
    if (topicName.empty())
    {
        return topicName;
    }

    if (topicName.rfind("rt/", 0) == 0)
    {
        return topicName;
    }

    if (topicName.front() == '/')
    {
        return "rt" + topicName;
    }

    return "rt/" + topicName;
}

}  // namespace sdk_event_bridge

#endif  // ROS_DDS_TOPIC_HPP
