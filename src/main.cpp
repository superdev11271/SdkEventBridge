#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "cmd_ctl_publisher.hpp"
#include "cmd_vel_publisher.hpp"
#include "sdk_event_bridge.hpp"

using namespace sdk_event_bridge;

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

static void HandleSportEvent(const std::string& eventName, const SportEventResult& result)
{
    if (result.IsSuccess())
    {
        std::cout << "[" << eventName << "] intercepted success (0)"
                  << " latency=" << result.responseLatencyMs << "ms"
                  << " requestId=" << result.requestId;
        if (!result.parameter.empty())
        {
            std::cout << " parameter=" << result.parameter;
        }
        std::cout << std::endl;
        return;
    }

    std::cout << "[" << eventName << "] intercepted failed (" << result.statusCode << ")"
              << " latency=" << result.responseLatencyMs << "ms"
              << " error=" << SdkEventBridgeClass::StatusCodeToString(result.statusCode)
              << " requestId=" << result.requestId << std::endl;
}

static void RegisterSportEventLogging(
    SdkEventBridgeClass& bridge,
    const std::shared_ptr<CmdVelPublisher>& cmdVelPublisher,
    const std::shared_ptr<CmdCtlPublisher>& cmdCtlPublisher)
{
    bridge.SetSportRequestHandler([](int32_t apiId, const unitree::robot::Request& request) {
        std::cout << "[intercept/request] api=" << SdkEventBridgeClass::SportApiIdToString(apiId)
                  << " id=" << request.header().identity().id()
                  << " parameter=" << request.parameter() << std::endl;
    });

    bridge.RegisterDampHandler([cmdVelPublisher, cmdCtlPublisher](const SportEventResult& result) {
        HandleSportEvent("DAMP", result);
        if (result.IsSuccess())
        {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->PublishMove(MoveVelocity{});
            }
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterMoveHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("MOVE", result);
        if (result.IsSuccess() && cmdVelPublisher)
        {
            cmdVelPublisher->PublishMoveFromParameter(result.parameter);
        }
    });

    bridge.RegisterBalanceStandHandler([cmdCtlPublisher](const SportEventResult& result) {
        HandleSportEvent("BALANCESTAND", result);
        if (result.IsSuccess())
        {
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterStopMoveHandler([cmdVelPublisher](const SportEventResult& result) {
        HandleSportEvent("STOPMOVE", result);
        if (result.IsSuccess() && cmdVelPublisher)
        {
            cmdVelPublisher->PublishMove(MoveVelocity{});
        }
    });

    bridge.RegisterStandUpHandler([cmdCtlPublisher](const SportEventResult& result) {
        HandleSportEvent("STANDUP", result);
        if (result.IsSuccess())
        {
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterStandDownHandler([cmdVelPublisher, cmdCtlPublisher](const SportEventResult& result) {
        HandleSportEvent("STANDDOWN", result);
        if (result.IsSuccess())
        {
            if (cmdVelPublisher)
            {
                cmdVelPublisher->PublishMove(MoveVelocity{});
            }
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterRecoveryStandHandler([cmdCtlPublisher](const SportEventResult& result) {
        HandleSportEvent("RECOVERYSTAND", result);
        if (result.IsSuccess())
        {
            PublishFsmCommandIfMapped(cmdCtlPublisher, result.apiId);
        }
    });

    bridge.RegisterFreeWalkHandler([](const SportEventResult& result) {
        HandleSportEvent("FREEWALK", result);
    });
}

static BridgeMode ParseMode(const std::string& mode)
{
    if (mode == "passive")
    {
        return BridgeMode::Passive;
    }

    return BridgeMode::Intercept;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <networkInterface> [domainId] [mode]" << std::endl;
        std::cout << "  mode: intercept (default) | passive" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        std::cout << "         " << argv[0] << " lo 0 intercept" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const std::string networkInterface = argv[1];
    int32_t domainId = 0;
    BridgeMode mode = BridgeMode::Intercept;

    if (argc > 2)
    {
        domainId = std::stoi(argv[2]);
    }
    if (argc > 3)
    {
        mode = ParseMode(argv[3]);
    }

    auto cmdVelPublisher = std::make_shared<CmdVelPublisher>("/cmd_vel");
    auto cmdCtlPublisher = std::make_shared<CmdCtlPublisher>("/cmd_ctl");

    SdkEventBridgeClass bridge(domainId, networkInterface);
    bridge.SetMode(mode);
    bridge.Init();
    RegisterSportEventLogging(bridge, cmdVelPublisher, cmdCtlPublisher);
    bridge.Start();

    std::cout << "SdkEventBridge mode="
              << (mode == BridgeMode::Intercept ? "intercept" : "passive")
              << " service=" << SPORT_SERVICE_NAME
              << " request=" << SPORT_REQUEST_TOPIC
              << " response=" << SPORT_RESPONSE_TOPIC
              << " ros2_cmd_vel=/cmd_vel"
              << " ros2_cmd_ctl=/cmd_ctl"
              << " domain=" << domainId
              << " interface=" << networkInterface << std::endl;

    if (mode == BridgeMode::Intercept)
    {
        std::cout << "Sport commands are handled locally and answered immediately (code 0)." << std::endl;
        std::cout << "MOVE -> /cmd_vel (geometry_msgs/Twist)" << std::endl;
        std::cout << "STANDUP/BALANCESTAND/RECOVERYSTAND -> /cmd_ctl 10001" << std::endl;
        std::cout << "STANDDOWN -> stop /cmd_vel, then /cmd_ctl 10002" << std::endl;
        std::cout << "DAMP -> stop /cmd_vel, then /cmd_ctl 10003" << std::endl;
        std::cout << "Keep the real robot off this DDS network to prevent it from executing commands." << std::endl;
    }

    while (rclcpp::ok())
    {
        cmdVelPublisher->SpinOnce();
        cmdCtlPublisher->SpinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bridge.Stop();
    rclcpp::shutdown();
    return 0;
}
