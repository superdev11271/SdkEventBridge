#ifndef CMD_VEL_PUBLISHER_HPP
#define CMD_VEL_PUBLISHER_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sdk_event_bridge
{

struct MoveVelocity
{
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
};

class CmdVelPublisher
{
public:
    static constexpr std::chrono::milliseconds MOVE_STOP_TIMEOUT{1000};

    explicit CmdVelPublisher(const std::string& topicName = "/cmd_vel");
    ~CmdVelPublisher();

    CmdVelPublisher(const CmdVelPublisher&) = delete;
    CmdVelPublisher& operator=(const CmdVelPublisher&) = delete;

    void SetContinuousMoveMode(bool enabled);
    bool IsContinuousMoveModeEnabled() const;

    void HandleMove(const std::string& parameterJson);
    void HandleStop();
    void PublishMove(const MoveVelocity& velocity);
    void PublishMoveFromParameter(const std::string& parameterJson);
    void Tick();
    void SpinOnce();

    static MoveVelocity ParseMoveParameter(const std::string& parameterJson);
    static bool ParseBoolParameter(const std::string& parameterJson, bool& value);

private:
    void PublishStop();

    rclcpp::Node::SharedPtr mNode;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPublisher;

    bool m_continuousMoveMode;
    bool m_hasActiveMove;
    MoveVelocity m_lastVelocity;
    std::chrono::steady_clock::time_point m_lastMoveTime;
};

}  // namespace sdk_event_bridge

#endif  // CMD_VEL_PUBLISHER_HPP
