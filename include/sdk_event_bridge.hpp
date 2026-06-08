#ifndef SDK_EVENT_BRIDGE_HPP
#define SDK_EVENT_BRIDGE_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <unitree/robot/b2/sport/sport_api.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_api.hpp>
#include <unitree/robot/internal/internal_request_response.hpp>
#include <unitree/robot/server/server_stub.hpp>

namespace sdk_event_bridge
{

inline constexpr const char* SPORT_REQUEST_TOPIC = "rt/api/sport/request";
inline constexpr const char* SPORT_RESPONSE_TOPIC = "rt/api/sport/response";
inline constexpr const char* SPORT_SERVICE_NAME = "sport";

inline constexpr int32_t SPORT_STATUS_SUCCESS = 0;

enum class BridgeMode
{
    Passive,
    Intercept,
};

enum class SportEventType : int32_t
{
    Damp = unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP,
    Move = unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE,
    BalanceStand = unitree::robot::go2::ROBOT_SPORT_API_ID_BALANCESTAND,
    StopMove = unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE,
    StandUp = unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP,
    StandDown = unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN,
    RecoveryStand = unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND,
    FreeWalk = unitree::robot::go2::ROBOT_SPORT_API_ID_FREEWALK,
    SwitchMoveMode = unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHMOVEMODE,
    SpeedLevel = unitree::robot::go2::ROBOT_SPORT_API_ID_SPEEDLEVEL,
    SwitchGait = unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHGAIT,
};

struct SportEventResult
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

using SportRequestHandler = std::function<void(int32_t apiId, const unitree::robot::Request& request)>;
using SportResponseHandler = std::function<void(int32_t apiId, const unitree::robot::Response& response)>;
using SportEventHandler = std::function<void(const SportEventResult& result)>;

class SdkEventBridgeClass
{
public:
    explicit SdkEventBridgeClass(int32_t domainId = 0, const std::string& networkInterface = "");
    ~SdkEventBridgeClass();

    SdkEventBridgeClass(const SdkEventBridgeClass&) = delete;
    SdkEventBridgeClass& operator=(const SdkEventBridgeClass&) = delete;

    void SetMode(BridgeMode mode);
    BridgeMode GetMode() const;

    void Init();
    void Start();
    void Stop();

    void SetSportRequestHandler(SportRequestHandler handler);
    void SetSportResponseHandler(SportResponseHandler handler);
    void SetSportEventHandler(SportEventHandler handler);

    void RegisterSportRequestHandler(int32_t apiId, SportRequestHandler handler);
    void RegisterSportResponseHandler(int32_t apiId, SportResponseHandler handler);
    void RegisterSportEventHandler(int32_t apiId, SportEventHandler handler);

    void RegisterDampHandler(SportEventHandler handler);
    void RegisterMoveHandler(SportEventHandler handler);
    void RegisterBalanceStandHandler(SportEventHandler handler);
    void RegisterStopMoveHandler(SportEventHandler handler);
    void RegisterStandUpHandler(SportEventHandler handler);
    void RegisterStandDownHandler(SportEventHandler handler);
    void RegisterRecoveryStandHandler(SportEventHandler handler);
    void RegisterFreeWalkHandler(SportEventHandler handler);
    void RegisterSwitchMoveModeHandler(SportEventHandler handler);
    void RegisterSpeedLevelHandler(SportEventHandler handler);
    void RegisterSwitchGaitHandler(SportEventHandler handler);

    static bool IsSuccess(int32_t statusCode);
    static std::string SportApiIdToString(int32_t apiId);
    static std::string StatusCodeToString(int32_t statusCode);

private:
    struct PendingRequest
    {
        int32_t apiId = 0;
        std::string parameter;
        std::chrono::steady_clock::time_point sentAt = std::chrono::steady_clock::now();
    };

    void StartPassive();
    void StartIntercept();
    void StopPassive();
    void StopIntercept();

    void OnSportRequest(const void* message);
    void OnSportResponse(const void* message);
    void HandleInterceptedRequest(const unitree::robot::RequestPtr& requestPtr);

    void DispatchRequest(int32_t apiId, const unitree::robot::Request& request);
    void DispatchResponse(int32_t apiId, const unitree::robot::Response& response);
    void DispatchEventResult(const SportEventResult& result);

    static bool IsTrackedSportApi(int32_t apiId);
    static std::string BuildInterceptResponseData(
        int32_t apiId,
        const unitree::robot::Request& request,
        int32_t& statusCode);
    static unitree::robot::Response BuildInterceptResponse(
        const unitree::robot::Request& request,
        int32_t statusCode,
        const std::string& data);

    BridgeMode mMode;
    int32_t mDomainId;
    std::string mNetworkInterface;
    bool mInitialized;
    bool mStarted;

    SportRequestHandler mDefaultRequestHandler;
    SportResponseHandler mDefaultResponseHandler;
    SportEventHandler mDefaultEventHandler;
    std::unordered_map<int32_t, SportRequestHandler> mRequestHandlers;
    std::unordered_map<int32_t, SportResponseHandler> mResponseHandlers;
    std::unordered_map<int32_t, SportEventHandler> mEventHandlers;
    std::unordered_map<int64_t, PendingRequest> mPendingRequests;

    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree::robot::Request>> mSportRequestSubscriber;
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree::robot::Response>> mSportResponseSubscriber;
    std::unique_ptr<unitree::robot::ServerStub> mSportServerStub;
};

}  // namespace sdk_event_bridge

#endif  // SDK_EVENT_BRIDGE_HPP
