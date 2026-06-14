#ifndef MOTION_SWITCHER_BRIDGE_HPP
#define MOTION_SWITCHER_BRIDGE_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <unitree/robot/b2/motion_switcher/motion_switcher_api.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/internal/internal_request_response.hpp>

#include "sdk_event_bridge.hpp"

namespace sdk_event_bridge
{

inline constexpr const char* MOTION_SWITCHER_REQUEST_TOPIC = "rt/api/motion_switcher/request";
inline constexpr const char* MOTION_SWITCHER_RESPONSE_TOPIC = "rt/api/motion_switcher/response";

enum class RobotPosture
{
    StandDown,
    StandUp,
    Walking,
};

struct MotionSwitcherEventResult
{
    int32_t apiId = 0;
    int64_t requestId = 0;
    int32_t statusCode = 0;
    int64_t responseLatencyMs = 0;
    std::string parameter;
    std::string data;

    bool IsSuccess() const
    {
        return statusCode == SPORT_STATUS_SUCCESS;
    }
};

using MotionSwitcherRequestHandler =
    std::function<void(int32_t apiId, const unitree::robot::Request& request)>;
using MotionSwitcherResponseHandler =
    std::function<void(int32_t apiId, const unitree::robot::Response& response)>;
using MotionSwitcherEventHandler = std::function<void(const MotionSwitcherEventResult& result)>;

class MotionSwitcherBridgeClass
{
public:
    explicit MotionSwitcherBridgeClass(int32_t domainId = 0, const std::string& networkInterface = "");
    ~MotionSwitcherBridgeClass();

    MotionSwitcherBridgeClass(const MotionSwitcherBridgeClass&) = delete;
    MotionSwitcherBridgeClass& operator=(const MotionSwitcherBridgeClass&) = delete;

    void Init();
    void Start();
    void Stop();

    void SetRequestHandler(MotionSwitcherRequestHandler handler);
    void SetResponseHandler(MotionSwitcherResponseHandler handler);
    void SetEventHandler(MotionSwitcherEventHandler handler);

    void RegisterRequestHandler(int32_t apiId, MotionSwitcherRequestHandler handler);
    void RegisterResponseHandler(int32_t apiId, MotionSwitcherResponseHandler handler);
    void RegisterEventHandler(int32_t apiId, MotionSwitcherEventHandler handler);

    void RegisterCheckModeHandler(MotionSwitcherEventHandler handler);
    void RegisterSelectModeHandler(MotionSwitcherEventHandler handler);
    void RegisterReleaseModeHandler(MotionSwitcherEventHandler handler);
    void RegisterSetSilentHandler(MotionSwitcherEventHandler handler);
    void RegisterGetSilentHandler(MotionSwitcherEventHandler handler);

    std::string GetForm() const;
    std::string GetModeName() const;
    bool GetSilent() const;

    void SetRobotPosture(RobotPosture posture);
    RobotPosture GetRobotPosture() const;
    bool CanChangeMotionMode() const;
    void ResetToDefaults();

    static bool IsTrackedApi(int32_t apiId);
    static std::string ApiIdToString(int32_t apiId);
    static const char* RobotPostureToString(RobotPosture posture);

private:
    void StartIntercept();
    void StopIntercept();

    void HandleInterceptedRequest(const unitree::robot::Request& request);

    void DispatchRequest(int32_t apiId, const unitree::robot::Request& request);
    void DispatchResponse(int32_t apiId, const unitree::robot::Response& response);
    void DispatchEventResult(const MotionSwitcherEventResult& result);

    std::string BuildInterceptResponseData(
        int32_t apiId,
        const unitree::robot::Request& request,
        int32_t& statusCode);
    static unitree::robot::Response BuildInterceptResponse(
        const unitree::robot::Request& request,
        int32_t statusCode,
        const std::string& data);

    int32_t mDomainId;
    std::string mNetworkInterface;
    bool mInitialized;
    bool mStarted;

    std::string mForm;
    std::string mModeName;
    bool mSilent;
    RobotPosture mRobotPosture;

    MotionSwitcherRequestHandler mDefaultRequestHandler;
    MotionSwitcherResponseHandler mDefaultResponseHandler;
    MotionSwitcherEventHandler mDefaultEventHandler;
    std::unordered_map<int32_t, MotionSwitcherRequestHandler> mRequestHandlers;
    std::unordered_map<int32_t, MotionSwitcherResponseHandler> mResponseHandlers;
    std::unordered_map<int32_t, MotionSwitcherEventHandler> mEventHandlers;

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree::robot::Request>> mRequestSubscriber;
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree::robot::Response>> mResponsePublisher;
};

}  // namespace sdk_event_bridge

#endif  // MOTION_SWITCHER_BRIDGE_HPP
