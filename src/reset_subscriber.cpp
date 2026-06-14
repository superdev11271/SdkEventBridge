#include "reset_subscriber.hpp"

#include <iostream>

#include "ros_dds_topic.hpp"

namespace sdk_event_bridge
{

ResetSubscriber::ResetSubscriber(const std::string& topicName)
    : mSubscriber(ToRosDdsTopicName(topicName)),
      m_channelInitialized(false)
{
}

ResetSubscriber::~ResetSubscriber()
{
    CloseChannel();
}

void ResetSubscriber::InitChannel(ResetHandler handler)
{
    if (m_channelInitialized || !handler)
    {
        return;
    }

    mSubscriber.InitChannel(
        [handler](const void* message) {
            const auto* pose = static_cast<const geometry_msgs::msg::dds_::Pose_*>(message);
            handler(*pose);
        },
        10);
    m_channelInitialized = true;
}

void ResetSubscriber::CloseChannel()
{
    if (!m_channelInitialized)
    {
        return;
    }

    mSubscriber.CloseChannel();
    m_channelInitialized = false;
}

const std::string& ResetSubscriber::GetChannelName() const
{
    return mSubscriber.GetChannelName();
}

}  // namespace sdk_event_bridge
