#ifndef CMD_CTL_PUBLISHER_HPP
#define CMD_CTL_PUBLISHER_HPP

#include <cstdint>
#include <string>

#include <unitree/idl/ros2/Int32_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

namespace sdk_event_bridge
{

enum class FsmCommand : int32_t
{
    GetUpLocomotion = 10001,
    GetDown = 10002,
    PassiveDamped = 10003,
};

class CmdCtlPublisher
{
public:
    explicit CmdCtlPublisher(const std::string& topicName = "/cmd_ctl");
    ~CmdCtlPublisher();

    CmdCtlPublisher(const CmdCtlPublisher&) = delete;
    CmdCtlPublisher& operator=(const CmdCtlPublisher&) = delete;

    void InitChannel();
    void PublishCommand(FsmCommand command);
    void PublishCommand(int32_t command);

    static std::string FsmCommandToString(FsmCommand command);
    static bool TryMapSportApiToFsmCommand(int32_t apiId, FsmCommand& command);

private:
    unitree::robot::ChannelPublisher<std_msgs::msg::dds_::Int32_> mPublisher;
    bool m_channelInitialized;
};

}  // namespace sdk_event_bridge

#endif  // CMD_CTL_PUBLISHER_HPP
