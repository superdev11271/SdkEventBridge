#include "cmd_ctl_publisher.hpp"

#include <iostream>

#include "ros_dds_topic.hpp"
#include <unitree/robot/go2/sport/sport_api.hpp>

namespace sdk_event_bridge
{

CmdCtlPublisher::CmdCtlPublisher(const std::string& topicName)
    : mPublisher(ToRosDdsTopicName(topicName)),
      m_channelInitialized(false)
{
}

CmdCtlPublisher::~CmdCtlPublisher()
{
    mPublisher.CloseChannel();
}

void CmdCtlPublisher::InitChannel()
{
    if (m_channelInitialized)
    {
        return;
    }

    mPublisher.InitChannel();
    m_channelInitialized = true;
}

void CmdCtlPublisher::PublishCommand(FsmCommand command)
{
    PublishCommand(static_cast<int32_t>(command));
}

void CmdCtlPublisher::PublishCommand(int32_t command)
{
    if (!m_channelInitialized)
    {
        InitChannel();
    }

    std_msgs::msg::dds_::Int32_ message;
    message.data() = command;
    const bool wrote = mPublisher.Write(message);

    std::cout << "[ctl] " << command << std::endl;
    if (!wrote)
    {
        std::cerr << "[ctl] publish failed" << std::endl;
    }
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
