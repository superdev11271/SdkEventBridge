#ifndef CMD_VEL_PUBLISHER_HPP
#define CMD_VEL_PUBLISHER_HPP

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
    explicit CmdVelPublisher(const std::string& topicName = "/cmd_vel");
    ~CmdVelPublisher();

    CmdVelPublisher(const CmdVelPublisher&) = delete;
    CmdVelPublisher& operator=(const CmdVelPublisher&) = delete;

    void PublishMove(const MoveVelocity& velocity);
    void PublishMoveFromParameter(const std::string& parameterJson);
    void SpinOnce();

    static MoveVelocity ParseMoveParameter(const std::string& parameterJson);

private:
    rclcpp::Node::SharedPtr mNode;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPublisher;
};

}  // namespace sdk_event_bridge

#endif  // CMD_VEL_PUBLISHER_HPP
