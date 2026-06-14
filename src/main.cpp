#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "cmd_ctl_publisher.hpp"
#include "cmd_vel_publisher.hpp"
#include "motion_switcher_bridge.hpp"
#include "reset_subscriber.hpp"
#include "sdk_event_bridge.hpp"

using namespace sdk_event_bridge;

namespace
{

std::atomic<bool> g_running{true};

void HandleShutdownSignal(int)
{
    g_running.store(false);
}

}  // namespace

static void PublishFsmCommandIfMapped(
    const std::shared_ptr<CmdCtlPublisher>& cmdCtlPublisher,
    int32_t apiId)
{
    if (!cmdCtlPublisher)
    {
        return;
    }

    FsmCommand command{};
    if (CmdCtlPublisher::TryMapSportApiToFsmCommand(apiId, command))
    {
        cmdCtlPublisher->PublishCommand(command);
    }
}

static void LogSportEvent(const std::string& eventName, const SportEventResult& result)
{
    if (result.IsSuccess())
    {
        if (eventName == "MOVE")
        {
            return;
        }

        std::cout << "> " << eventName;
        if (!result.parameter.empty())
        {
            std::cout << ' ' << result.parameter;
        }
        std::cout << std::endl;
        return;
    }

    std::cout << "! " << eventName << " err=" << result.statusCode << std::endl;
}

static void LogMotionSwitcherEvent(const std::string& eventName, const MotionSwitcherEventResult& result)
{
    if (result.IsSuccess())
    {
        if (eventName == "CHECK_MODE" || eventName == "GET_SILENT")
        {
            return;
        }

        std::cout << "> " << eventName;
        if (!result.parameter.empty())
        {
            std::cout << ' ' << result.parameter;
        }
        std::cout << std::endl;
        return;
    }

    std::cout << "! " << eventName << " err=" << result.statusCode << std::endl;
}

static void HandleMotionSwitcherEvent(
    const std::string& eventName,
    const MotionSwitcherEventResult& result)
{
    LogMotionSwitcherEvent(eventName, result);
}

static void RegisterMotionSwitcherEventLogging(
    MotionSwitcherBridgeClass& bridge,
    const std::shared_ptr<CmdVelPublisher>& cmdVelPublisher)
{
    bridge.RegisterCheckModeHandler([](const MotionSwitcherEventResult& result) {
        HandleMotionSwitcherEvent("CHECK_MODE", result);
    });

    bridge.RegisterSelectModeHandler([cmdVelPublisher](const MotionSwitcherEventResult& result) {
        HandleMotionSwitcherEvent("SELECT_MODE", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        std::string modeName;
        if (CmdVelPublisher::ParseStringParameter(result.parameter, "name", modeName))
        {
            cmdVelPublisher->SetMotionModeName(modeName);
        }
    });

    bridge.RegisterReleaseModeHandler([cmdVelPublisher](const MotionSwitcherEventResult& result) {
        HandleMotionSwitcherEvent("RELEASE_MODE", result);
        if (result.IsSuccess() && cmdVelPublisher)
        {
            cmdVelPublisher->SetMotionModeName("ai");
        }
    });

    bridge.RegisterSetSilentHandler([](const MotionSwitcherEventResult& result) {
        HandleMotionSwitcherEvent("SET_SILENT", result);
    });

    bridge.RegisterGetSilentHandler([](const MotionSwitcherEventResult& result) {
        HandleMotionSwitcherEvent("GET_SILENT", result);
    });
}

static void HandleSportEvent(const std::string& eventName, const SportEventResult& result)
{
    LogSportEvent(eventName, result);
}

static void UpdatePostureFromMove(
    MotionSwitcherBridgeClass& motionSwitcherBridge,
    const std::shared_ptr<CmdVelPublisher>& cmdVelPublisher,
    const std::string& parameterJson)
{
    if (!cmdVelPublisher || cmdVelPublisher->IsJointsLocked())
    {
        return;
    }

    const MoveVelocity velocity = CmdVelPublisher::ParseMoveParameter(parameterJson);
    const bool hasMotion =
        std::abs(velocity.vx) > 0.0 || std::abs(velocity.vy) > 0.0 || std::abs(velocity.vyaw) > 0.0;
    if (hasMotion)
    {
        motionSwitcherBridge.SetRobotPosture(RobotPosture::Walking);
    }
}

static void RegisterSportEventLogging(
    SdkEventBridgeClass& bridge,
    MotionSwitcherBridgeClass& motionSwitcherBridge,
    const std::shared_ptr<CmdVelPublisher>& cmdVelPublisher,
    const std::shared_ptr<CmdCtlPublisher>& cmdCtlPublisher)
{
    bridge.RegisterSportRequestHandler(
        unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP,
        [&motionSwitcherBridge, cmdVelPublisher, cmdCtlPublisher](
            int32_t apiId, const unitree::robot::Request&) {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->LockJoints();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandUp);
            PublishFsmCommandIfMapped(cmdCtlPublisher, apiId);
        });

    bridge.RegisterSportRequestHandler(
        unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN,
        [&motionSwitcherBridge, cmdVelPublisher, cmdCtlPublisher](
            int32_t apiId, const unitree::robot::Request&) {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->LockJoints();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandDown);
            PublishFsmCommandIfMapped(cmdCtlPublisher, apiId);
        });

    bridge.RegisterSportRequestHandler(
        unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND,
        [&motionSwitcherBridge, cmdVelPublisher, cmdCtlPublisher](
            int32_t apiId, const unitree::robot::Request&) {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->LockJoints();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandUp);
            PublishFsmCommandIfMapped(cmdCtlPublisher, apiId);
        });

    bridge.RegisterDampHandler([cmdVelPublisher, cmdCtlPublisher, &motionSwitcherBridge](
        const SportEventResult& result) {
        HandleSportEvent("DAMP", result);
        if (result.IsSuccess())
        {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->LockJoints();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandDown);
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterMoveHandler([&motionSwitcherBridge, cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("MOVE", result);
        if (result.IsSuccess() && cmdVelPublisher)
        {
            cmdVelPublisher->HandleMove(result.parameter);
            UpdatePostureFromMove(motionSwitcherBridge, cmdVelPublisher, result.parameter);
        }
    });

    bridge.RegisterSwitchMoveModeHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("SWITCHMOVEMODE", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        bool enabled = false;
        if (CmdVelPublisher::ParseBoolParameter(result.parameter, enabled))
        {
            cmdVelPublisher->SetContinuousMoveMode(enabled);
        }
    });

    bridge.RegisterSpeedLevelHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("SPEEDLEVEL", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        int level = -1;
        if (CmdVelPublisher::ParseIntParameter(result.parameter, level))
        {
            cmdVelPublisher->SetSpeedLevel(level);
        }
    });

    bridge.RegisterBalanceStandHandler([&motionSwitcherBridge, cmdVelPublisher, cmdCtlPublisher](
        const SportEventResult& result) {
        HandleSportEvent("BALANCESTAND", result);
        if (result.IsSuccess())
        {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->UnlockJoints();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandUp);
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterSwitchGaitHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("SWITCHGAIT", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        int gait = 0;
        if (CmdVelPublisher::ParseIntParameter(result.parameter, gait))
        {
            cmdVelPublisher->SetGait(gait);
        }
    });

    bridge.RegisterStopMoveHandler([&motionSwitcherBridge, cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("STOPMOVE", result);
        if (result.IsSuccess())
        {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->HandleStop();
            }
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandUp);
        }
    });

    bridge.RegisterStandUpHandler([](const SportEventResult& result) {
        HandleSportEvent("STANDUP", result);
    });

    bridge.RegisterStandDownHandler([](const SportEventResult& result) {
        HandleSportEvent("STANDDOWN", result);
    });

    bridge.RegisterRecoveryStandHandler([](const SportEventResult& result) {
        HandleSportEvent("RECOVERYSTAND", result);
    });

    bridge.RegisterFreeWalkHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("FREEWALK", result);
        if (result.IsSuccess() && cmdVelPublisher)
        {
            cmdVelPublisher->SetFreeWalk();
        }
    });

    bridge.RegisterVisionWalkHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("VISIONWALK", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        bool enabled = false;
        if (CmdVelPublisher::ParseBoolParameter(result.parameter, enabled))
        {
            cmdVelPublisher->SetVisionWalk(enabled);
        }
    });

    bridge.RegisterClassicWalkHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("CLASSICWALK", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        bool enabled = false;
        if (CmdVelPublisher::ParseBoolParameter(result.parameter, enabled))
        {
            cmdVelPublisher->SetClassicWalk(enabled);
        }
    });

    bridge.RegisterFastWalkHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("FASTWALK", result);
        if (!result.IsSuccess() || !cmdVelPublisher)
        {
            return;
        }

        bool enabled = false;
        if (CmdVelPublisher::ParseBoolParameter(result.parameter, enabled))
        {
            cmdVelPublisher->SetFastWalk(enabled);
        }
    });
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <networkInterface>" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return 1;
    }

    std::signal(SIGINT, HandleShutdownSignal);
    std::signal(SIGTERM, HandleShutdownSignal);

    const std::string networkInterface = argv[1];
    constexpr int32_t domainId = 0;

    auto cmdVelPublisher = std::make_shared<CmdVelPublisher>("/cmd_vel_origin");
    auto cmdCtlPublisher = std::make_shared<CmdCtlPublisher>("/cmd_ctl");

    SdkEventBridgeClass sportBridge(domainId, networkInterface);
    MotionSwitcherBridgeClass motionSwitcherBridge(domainId, networkInterface);

    sportBridge.Init();
    motionSwitcherBridge.Init();
    cmdVelPublisher->InitChannel();
    cmdCtlPublisher->InitChannel();

    ResetSubscriber resetSubscriber("/reset");
    resetSubscriber.InitChannel([&cmdVelPublisher, &motionSwitcherBridge](
        const geometry_msgs::msg::dds_::Pose_&) {
        std::cout << "> RESET" << std::endl;
        if (cmdVelPublisher)
        {
            cmdVelPublisher->ResetToDefaults();
        }
        motionSwitcherBridge.ResetToDefaults();
    });

    RegisterSportEventLogging(sportBridge, motionSwitcherBridge, cmdVelPublisher, cmdCtlPublisher);
    RegisterMotionSwitcherEventLogging(motionSwitcherBridge, cmdVelPublisher);

    sportBridge.Start();
    motionSwitcherBridge.Start();

    std::cout << "SdkEventBridge on " << networkInterface
              << " | /cmd_vel_origin /cmd_ctl /reset" << std::endl;

    while (g_running.load())
    {
        cmdVelPublisher->Tick();
        if (cmdVelPublisher->IsActivelyMoving())
        {
            motionSwitcherBridge.SetRobotPosture(RobotPosture::Walking);
        }
        else if (motionSwitcherBridge.GetRobotPosture() == RobotPosture::Walking)
        {
            motionSwitcherBridge.SetRobotPosture(RobotPosture::StandUp);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    sportBridge.Stop();
    motionSwitcherBridge.Stop();
    cmdVelPublisher.reset();
    cmdCtlPublisher.reset();
    return 0;
}
