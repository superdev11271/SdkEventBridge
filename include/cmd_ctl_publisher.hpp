#ifndef CMD_CTL_PUBLISHER_HPP
#define CMD_CTL_PUBLISHER_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

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

    void PublishCommand(FsmCommand command);
    void PublishCommand(int32_t command);
    void SpinOnce();

    static std::string FsmCommandToString(FsmCommand command);
    static bool TryMapSportApiToFsmCommand(int32_t apiId, FsmCommand& command);

private:
    rclcpp::Node::SharedPtr mNode;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mPublisher;
};

}  // namespace sdk_event_bridge

#endif  // CMD_CTL_PUBLISHER_HPP
