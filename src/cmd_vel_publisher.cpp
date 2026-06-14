#include "cmd_vel_publisher.hpp"

#include <cmath>
#include <iostream>

#include "ros_dds_topic.hpp"
#include <unitree/common/json/jsonize.hpp>

namespace sdk_event_bridge
{

CmdVelPublisher::CmdVelPublisher(const std::string& topicName)
    : mPublisher(ToRosDdsTopicName(topicName)),
      m_channelInitialized(false),
      m_continuousMoveMode(false),
      m_jointsLocked(true),
      m_walkMode(WalkMode::None),
      m_hasActiveMove(false),
      m_motionProfile(MotionProfile::Ai),
      m_speedLevel(-1),
      m_maxLinearSpeed(AI_LOW_MAX_LINEAR_SPEED)
{
}

CmdVelPublisher::~CmdVelPublisher()
{
    mPublisher.CloseChannel();
}

void CmdVelPublisher::InitChannel()
{
    if (m_channelInitialized)
    {
        return;
    }

    mPublisher.InitChannel();
    m_channelInitialized = true;
}

void CmdVelPublisher::SetContinuousMoveMode(bool enabled)
{
    m_continuousMoveMode = enabled;

    if (!enabled)
    {
        m_lastMoveTime = std::chrono::steady_clock::now();
    }
}

bool CmdVelPublisher::IsContinuousMoveModeEnabled() const
{
    return m_continuousMoveMode;
}

void CmdVelPublisher::SetSpeedLevel(int level)
{
    m_speedLevel = level;

    if (m_motionProfile == MotionProfile::Sport)
    {
        return;
    }

    ApplyMaxLinearSpeed();

    if (!m_jointsLocked && m_hasActiveMove)
    {
        m_lastVelocity = ApplySpeedLimit(m_lastVelocity);
        PublishMove(m_lastVelocity);
    }
}

MotionProfile CmdVelPublisher::MotionProfileFromModeName(const std::string& name)
{
    if (name == "normal" || name == "normal-w")
    {
        return MotionProfile::Sport;
    }

    return MotionProfile::Ai;
}

const char* CmdVelPublisher::MotionProfileToString(MotionProfile profile)
{
    switch (profile)
    {
    case MotionProfile::Sport:
        return "sport";
    case MotionProfile::Ai:
    default:
        return "ai";
    }
}

void CmdVelPublisher::ApplyMaxLinearSpeed()
{
    if (m_motionProfile == MotionProfile::Sport)
    {
        m_maxLinearSpeed = SPORT_MAX_LINEAR_SPEED;
        return;
    }

    m_maxLinearSpeed = m_speedLevel == 1 ? AI_HIGH_MAX_LINEAR_SPEED : AI_LOW_MAX_LINEAR_SPEED;
}

void CmdVelPublisher::SetMotionModeName(const std::string& name)
{
    const MotionProfile profile = MotionProfileFromModeName(name);
    if (m_motionProfile == profile)
    {
        return;
    }

    m_motionProfile = profile;
    std::cout << "[vel] profile " << MotionProfileToString(profile) << std::endl;
    ApplyMaxLinearSpeed();

    if (!m_jointsLocked && m_hasActiveMove)
    {
        m_lastVelocity = ApplySpeedLimit(m_lastVelocity);
        PublishMove(m_lastVelocity);
    }
}

MotionProfile CmdVelPublisher::GetMotionProfile() const
{
    return m_motionProfile;
}

void CmdVelPublisher::ResetToDefaults()
{
    m_continuousMoveMode = false;
    m_jointsLocked = true;
    m_walkMode = WalkMode::None;
    m_hasActiveMove = false;
    m_motionProfile = MotionProfile::Ai;
    m_speedLevel = -1;
    m_maxLinearSpeed = AI_LOW_MAX_LINEAR_SPEED;
    m_lastVelocity = MoveVelocity{};
    m_lastMoveTime = std::chrono::steady_clock::now();
    PublishStop();
}

void CmdVelPublisher::LockJoints()
{
    m_jointsLocked = true;
    m_hasActiveMove = false;
    PublishStop();
}

void CmdVelPublisher::UnlockJoints()
{
    m_jointsLocked = false;
}

void CmdVelPublisher::SetGait(int gait)
{
    if (gait == 0)
    {
        LockJoints();
        return;
    }

    if (gait >= 1 && gait <= 4)
    {
        UnlockJoints();
        return;
    }
}

bool CmdVelPublisher::IsJointsLocked() const
{
    return m_jointsLocked;
}

bool CmdVelPublisher::IsActivelyMoving() const
{
    return m_hasActiveMove && !m_jointsLocked;
}

const char* CmdVelPublisher::WalkModeToString(WalkMode mode)
{
    switch (mode)
    {
    case WalkMode::VisionWalk:
        return "vision walk";
    case WalkMode::FreeWalk:
        return "free walk";
    case WalkMode::ClassicWalk:
        return "classic walk";
    case WalkMode::FastWalk:
        return "fast walk";
    case WalkMode::None:
    default:
        return "none";
    }
}

void CmdVelPublisher::SetWalkMode(WalkMode mode, bool enabled)
{
    if (enabled)
    {
        m_walkMode = mode;
        UnlockJoints();
        return;
    }

    if (m_walkMode == mode)
    {
        m_walkMode = WalkMode::None;
        LockJoints();
    }
}

void CmdVelPublisher::SetVisionWalk(bool enabled)
{
    SetWalkMode(WalkMode::VisionWalk, enabled);
}

void CmdVelPublisher::SetFreeWalk()
{
    m_walkMode = WalkMode::FreeWalk;
    UnlockJoints();
}

void CmdVelPublisher::SetClassicWalk(bool enabled)
{
    SetWalkMode(WalkMode::ClassicWalk, enabled);
}

void CmdVelPublisher::SetFastWalk(bool enabled)
{
    SetWalkMode(WalkMode::FastWalk, enabled);
}

WalkMode CmdVelPublisher::GetWalkMode() const
{
    return m_walkMode;
}

double CmdVelPublisher::GetMaxLinearSpeed() const
{
    return m_maxLinearSpeed;
}

bool CmdVelPublisher::ParseIntParameter(const std::string& parameterJson, int& value)
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
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed to parse int parameter: " << ex.what() << std::endl;
    }

    return false;
}

bool CmdVelPublisher::ParseStringParameter(
    const std::string& parameterJson,
    const std::string& key,
    std::string& value)
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
        const auto keyIt = jsonMap.find(key);
        if (keyIt != jsonMap.end())
        {
            unitree::common::FromJson(keyIt->second, value);
            return true;
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Failed to parse string parameter: " << ex.what() << std::endl;
    }

    return false;
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

MoveVelocity CmdVelPublisher::ApplySpeedLimit(const MoveVelocity& velocity) const
{
    MoveVelocity limited = velocity;
    const double magnitude = std::hypot(limited.vx, limited.vy);
    if (magnitude > m_maxLinearSpeed && magnitude > 0.0)
    {
        const double scale = m_maxLinearSpeed / magnitude;
        limited.vx *= scale;
        limited.vy *= scale;
    }

    return limited;
}

void CmdVelPublisher::PublishMove(const MoveVelocity& velocity)
{
    geometry_msgs::msg::dds_::Twist_ twist;
    twist.linear().x() = velocity.vx;
    twist.linear().y() = velocity.vy;
    twist.linear().z() = 0.0;
    twist.angular().x() = 0.0;
    twist.angular().y() = 0.0;
    twist.angular().z() = velocity.vyaw;

    if (!m_channelInitialized)
    {
        InitChannel();
    }

    const bool wrote = mPublisher.Write(twist);

    const bool isStop =
        std::abs(velocity.vx) < 1e-9 && std::abs(velocity.vy) < 1e-9 && std::abs(velocity.vyaw) < 1e-9;
    if (!isStop)
    {
        std::cout << "[vel] " << velocity.vx << ' ' << velocity.vy << ' ' << velocity.vyaw << std::endl;
    }
    else if (!wrote)
    {
        std::cerr << "[vel] publish failed" << std::endl;
    }
}

void CmdVelPublisher::PublishStop()
{
    PublishMove(MoveVelocity{});
}

void CmdVelPublisher::HandleMove(const std::string& parameterJson)
{
    if (m_jointsLocked)
    {
        return;
    }

    m_lastVelocity = ApplySpeedLimit(ParseMoveParameter(parameterJson));
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
    if (m_jointsLocked || m_continuousMoveMode || !m_hasActiveMove)
    {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - m_lastMoveTime;
    if (elapsed < MOVE_STOP_TIMEOUT)
    {
        return;
    }

    m_hasActiveMove = false;
    PublishStop();
}

}  // namespace sdk_event_bridge
