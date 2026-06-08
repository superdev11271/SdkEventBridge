#include "cmd_ctl_publisher.hpp"

#include <iostream>

#include <unitree/robot/go2/sport/sport_api.hpp>

namespace sdk_event_bridge
{

CmdCtlPublisher::CmdCtlPublisher(const std::string& topicName)
    : mNode(std::make_shared<rclcpp::Node>("sdk_event_bridge_cmd_ctl"))
{
    mPublisher = mNode->create_publisher<std_msgs::msg::Int32>(topicName, 10);
}

CmdCtlPublisher::~CmdCtlPublisher() = default;

void CmdCtlPublisher::PublishCommand(FsmCommand command)
{
    PublishCommand(static_cast<int32_t>(command));
}

void CmdCtlPublisher::PublishCommand(int32_t command)
{
    std_msgs::msg::Int32 message;
    message.data = command;
    mPublisher->publish(message);

    std::cout << "[cmd_ctl] published data=" << command
              << " (" << FsmCommandToString(static_cast<FsmCommand>(command)) << ")"
              << std::endl;
}

void CmdCtlPublisher::SpinOnce()
{
    rclcpp::spin_some(mNode);
}

std::string CmdCtlPublisher::FsmCommandToString(FsmCommand command)
{
    switch (command)
    {
    case FsmCommand::GetUpLocomotion:
        return "Get up -> locomotion";
    case FsmCommand::GetDown:
        return "Get down";
    case FsmCommand::PassiveDamped:
        return "Passive (damped motors)";
    default:
        return "Unknown FSM command";
    }
}

bool CmdCtlPublisher::TryMapSportApiToFsmCommand(int32_t apiId, FsmCommand& command)
{
    switch (apiId)
    {
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_BALANCESTAND:
        command = FsmCommand::GetUpLocomotion;
        return true;
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN:
        command = FsmCommand::GetDown;
        return true;
    case unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP:
        command = FsmCommand::PassiveDamped;
        return true;
    default:
        return false;
    }
}

}  // namespace sdk_event_bridge
