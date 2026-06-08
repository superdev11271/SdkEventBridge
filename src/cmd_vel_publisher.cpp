#include "cmd_vel_publisher.hpp"

#include <iostream>

#include <unitree/common/json/jsonize.hpp>

namespace sdk_event_bridge
{

CmdVelPublisher::CmdVelPublisher(const std::string& topicName)
    : mNode(std::make_shared<rclcpp::Node>("sdk_event_bridge_cmd_vel")),
      m_continuousMoveMode(false),
      m_hasActiveMove(false)
{
    mPublisher = mNode->create_publisher<geometry_msgs::msg::Twist>(topicName, 10);
}

CmdVelPublisher::~CmdVelPublisher() = default;

void CmdVelPublisher::SetContinuousMoveMode(bool enabled)
{
    m_continuousMoveMode = enabled;
    std::cout << "[move_mode] continuous response "
              << (enabled ? "enabled" : "disabled") << std::endl;

    if (!enabled)
    {
        m_lastMoveTime = std::chrono::steady_clock::now();
    }
}

bool CmdVelPublisher::IsContinuousMoveModeEnabled() const
{
    return m_continuousMoveMode;
}

bool CmdVelPublisher::ParseBoolParameter(const std::string& parameterJson, bool& value)
{
    if (parameterJson.empty())
    {
        return false;
    }

    try
    {
        const unitree::common::Any jsonAny = unitree::common::FromJsonString(parameterJson);
        if (!unitree::common::IsJsonMap(jsonAny))
        {
            return false;
        }

        const unitree::common::JsonMap& jsonMap = unitree::common::AnyCast<unitree::common::JsonMap>(jsonAny);
        const auto dataIt = jsonMap.find("data");
        if (dataIt != jsonMap.end())
        {
            unitree::common::FromJson(dataIt->second, value);
            return true;
        }

        const auto flagIt = jsonMap.find("flag");
        if (flagIt != jsonMap.end())
        {
            unitree::common::FromJson(flagIt->second, value);
            return true;
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed to parse bool parameter: " << ex.what() << std::endl;
    }

    return false;
}

MoveVelocity CmdVelPublisher::ParseMoveParameter(const std::string& parameterJson)
{
    MoveVelocity velocity;

    if (parameterJson.empty())
    {
        return velocity;
    }

    try
    {
        const unitree::common::Any jsonAny = unitree::common::FromJsonString(parameterJson);
        if (!unitree::common::IsJsonMap(jsonAny))
        {
            return velocity;
        }

        const unitree::common::JsonMap& jsonMap = unitree::common::AnyCast<unitree::common::JsonMap>(jsonAny);
        const auto xIt = jsonMap.find("x");
        const auto yIt = jsonMap.find("y");
        const auto zIt = jsonMap.find("z");
        if (xIt != jsonMap.end())
        {
            unitree::common::FromJson(xIt->second, velocity.vx);
        }
        if (yIt != jsonMap.end())
        {
            unitree::common::FromJson(yIt->second, velocity.vy);
        }
        if (zIt != jsonMap.end())
        {
            unitree::common::FromJson(zIt->second, velocity.vyaw);
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed to parse move parameter: " << ex.what() << std::endl;
    }

    return velocity;
}

void CmdVelPublisher::PublishMove(const MoveVelocity& velocity)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity.vx;
    twist.linear.y = velocity.vy;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = velocity.vyaw;

    mPublisher->publish(twist);

    std::cout << "[cmd_vel] published linear=(" << twist.linear.x << ", "
              << twist.linear.y << ", " << twist.linear.z << ") angular=("
              << twist.angular.x << ", " << twist.angular.y << ", "
              << twist.angular.z << ")" << std::endl;
}

void CmdVelPublisher::PublishStop()
{
    PublishMove(MoveVelocity{});
}

void CmdVelPublisher::HandleMove(const std::string& parameterJson)
{
    m_lastVelocity = ParseMoveParameter(parameterJson);
    m_lastMoveTime = std::chrono::steady_clock::now();
    m_hasActiveMove = true;
    PublishMove(m_lastVelocity);
}

void CmdVelPublisher::HandleStop()
{
    m_hasActiveMove = false;
    PublishStop();
}

void CmdVelPublisher::PublishMoveFromParameter(const std::string& parameterJson)
{
    HandleMove(parameterJson);
}

void CmdVelPublisher::Tick()
{
    if (m_continuousMoveMode || !m_hasActiveMove)
    {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - m_lastMoveTime;
    if (elapsed < MOVE_STOP_TIMEOUT)
    {
        return;
    }

    std::cout << "[move_mode] no new MOVE for 1s, auto stop" << std::endl;
    m_hasActiveMove = false;
    PublishStop();
}

void CmdVelPublisher::SpinOnce()
{
    rclcpp::spin_some(mNode);
}

}  // namespace sdk_event_bridge
