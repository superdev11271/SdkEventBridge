#ifndef CMD_VEL_PUBLISHER_HPP
#define CMD_VEL_PUBLISHER_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

namespace sdk_event_bridge
{

struct MoveVelocity
{
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
};

enum class WalkMode
{
    None,
    VisionWalk,
    FreeWalk,
    ClassicWalk,
    FastWalk,
};

enum class MotionProfile
{
    Ai,
    Sport,
};

class CmdVelPublisher
{
public:
    static constexpr std::chrono::milliseconds MOVE_STOP_TIMEOUT{1000};
    static constexpr double AI_LOW_MAX_LINEAR_SPEED = 1.5;
    static constexpr double AI_HIGH_MAX_LINEAR_SPEED = 3.5;
    static constexpr double SPORT_MAX_LINEAR_SPEED = 6.0;

    // Backward-compatible aliases for AI mode limits.
    static constexpr double LOW_MAX_LINEAR_SPEED = AI_LOW_MAX_LINEAR_SPEED;
    static constexpr double HIGH_MAX_LINEAR_SPEED = AI_HIGH_MAX_LINEAR_SPEED;

    explicit CmdVelPublisher(const std::string& topicName = "/cmd_vel");
    ~CmdVelPublisher();

    CmdVelPublisher(const CmdVelPublisher&) = delete;
    CmdVelPublisher& operator=(const CmdVelPublisher&) = delete;

    void InitChannel();
    void SetContinuousMoveMode(bool enabled);
    bool IsContinuousMoveModeEnabled() const;
    void SetSpeedLevel(int level);
    void SetMotionModeName(const std::string& name);
    MotionProfile GetMotionProfile() const;
    double GetMaxLinearSpeed() const;

    void LockJoints();
    void UnlockJoints();
    void SetGait(int gait);
    bool IsJointsLocked() const;
    bool IsActivelyMoving() const;

    void SetVisionWalk(bool enabled);
    void SetFreeWalk();
    void SetClassicWalk(bool enabled);
    void SetFastWalk(bool enabled);
    WalkMode GetWalkMode() const;

    void ResetToDefaults();
    void HandleMove(const std::string& parameterJson);
    void HandleStop();
    void PublishMove(const MoveVelocity& velocity);
    void PublishMoveFromParameter(const std::string& parameterJson);
    void Tick();

    static MoveVelocity ParseMoveParameter(const std::string& parameterJson);
    static bool ParseBoolParameter(const std::string& parameterJson, bool& value);
    static bool ParseIntParameter(const std::string& parameterJson, int& value);
    static bool ParseStringParameter(const std::string& parameterJson, const std::string& key, std::string& value);

private:
    void SetWalkMode(WalkMode mode, bool enabled);
    static const char* WalkModeToString(WalkMode mode);
    static MotionProfile MotionProfileFromModeName(const std::string& name);
    static const char* MotionProfileToString(MotionProfile profile);
    void ApplyMaxLinearSpeed();

    MoveVelocity ApplySpeedLimit(const MoveVelocity& velocity) const;
    void PublishStop();

    unitree::robot::ChannelPublisher<geometry_msgs::msg::dds_::Twist_> mPublisher;

    bool m_channelInitialized;
    bool m_continuousMoveMode;
    bool m_jointsLocked;
    WalkMode m_walkMode;
    bool m_hasActiveMove;
    MotionProfile m_motionProfile;
    int m_speedLevel;
    double m_maxLinearSpeed;
    MoveVelocity m_lastVelocity;
    std::chrono::steady_clock::time_point m_lastMoveTime;
};

}  // namespace sdk_event_bridge

#endif  // CMD_VEL_PUBLISHER_HPP
