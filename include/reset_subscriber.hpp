#ifndef RESET_SUBSCRIBER_HPP
#define RESET_SUBSCRIBER_HPP

#include <functional>
#include <string>

#include <unitree/idl/ros2/Pose_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace sdk_event_bridge
{

class ResetSubscriber
{
public:
    using ResetHandler = std::function<void(const geometry_msgs::msg::dds_::Pose_& pose)>;

    explicit ResetSubscriber(const std::string& topicName = "/reset");
    ~ResetSubscriber();

    ResetSubscriber(const ResetSubscriber&) = delete;
    ResetSubscriber& operator=(const ResetSubscriber&) = delete;

    void InitChannel(ResetHandler handler);
    void CloseChannel();

    const std::string& GetChannelName() const;

private:
    unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Pose_> mSubscriber;
    bool m_channelInitialized;
};

}  // namespace sdk_event_bridge

#endif  // RESET_SUBSCRIBER_HPP
