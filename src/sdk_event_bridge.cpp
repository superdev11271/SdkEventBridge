#include "sdk_event_bridge.hpp"

#include <thread>

#include "channel_factory_init.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_error.hpp>
#include <unitree/robot/internal/internal_api.hpp>
#include <unitree/robot/internal/internal_error.hpp>

namespace sdk_event_bridge
{

SdkEventBridgeClass::SdkEventBridgeClass(int32_t domainId, const std::string& networkInterface)
    : mDomainId(domainId),
      mNetworkInterface(networkInterface),
      mInitialized(false),
      mStarted(false)
{
}

SdkEventBridgeClass::~SdkEventBridgeClass()
{
    Stop();
}

void SdkEventBridgeClass::Init()
{
    if (mInitialized)
    {
        return;
    }

    InitUnitreeChannelFactoryOnce(mDomainId, mNetworkInterface);
    mInitialized = true;
}

void SdkEventBridgeClass::Start()
{
    if (!mInitialized)
    {
        Init();
    }

    if (mStarted)
    {
        return;
    }

    StartIntercept();
    mStarted = true;
}

void SdkEventBridgeClass::Stop()
{
    StopIntercept();
    mStarted = false;
}

void SdkEventBridgeClass::StartIntercept()
{
    mSportServerStub = std::make_unique<unitree::robot::ServerStub>();
    mSportServerStub->Init(
        SPORT_SERVICE_NAME,
        [this](const unitree::robot::RequestPtr& requestPtr) {
            HandleInterceptedRequest(requestPtr);
        },
        false);
}

void SdkEventBridgeClass::StopIntercept()
{
    mSportServerStub.reset();
}

void SdkEventBridgeClass::SetSportRequestHandler(SportRequestHandler handler)
{
    mDefaultRequestHandler = std::move(handler);
}

void SdkEventBridgeClass::SetSportResponseHandler(SportResponseHandler handler)
{
    mDefaultResponseHandler = std::move(handler);
}

void SdkEventBridgeClass::SetSportEventHandler(SportEventHandler handler)
{
    mDefaultEventHandler = std::move(handler);
}

void SdkEventBridgeClass::RegisterSportRequestHandler(int32_t apiId, SportRequestHandler handler)
{
    mRequestHandlers[apiId] = std::move(handler);
}

void SdkEventBridgeClass::RegisterSportResponseHandler(int32_t apiId, SportResponseHandler handler)
{
    mResponseHandlers[apiId] = std::move(handler);
}

void SdkEventBridgeClass::RegisterSportEventHandler(int32_t apiId, SportEventHandler handler)
{
    mEventHandlers[apiId] = std::move(handler);
}

void SdkEventBridgeClass::RegisterDampHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP, std::move(handler));
}

void SdkEventBridgeClass::RegisterMoveHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE, std::move(handler));
}

void SdkEventBridgeClass::RegisterBalanceStandHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_BALANCESTAND, std::move(handler));
}

void SdkEventBridgeClass::RegisterStopMoveHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE, std::move(handler));
}

void SdkEventBridgeClass::RegisterStandUpHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP, std::move(handler));
}

void SdkEventBridgeClass::RegisterStandDownHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN, std::move(handler));
}

void SdkEventBridgeClass::RegisterRecoveryStandHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND, std::move(handler));
}

void SdkEventBridgeClass::RegisterFreeWalkHandler(SportEventHandler handler)
{
    const SportEventHandler sharedHandler = handler;
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_FREEWALK, sharedHandler);
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_FREEWALK, sharedHandler);
}

void SdkEventBridgeClass::RegisterVisionWalkHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_VISIONWALK, std::move(handler));
}

void SdkEventBridgeClass::RegisterClassicWalkHandler(SportEventHandler handler)
{
    const SportEventHandler sharedHandler = handler;
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_CLASSICWALK, sharedHandler);
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_CLASSICWALK, sharedHandler);
}

void SdkEventBridgeClass::RegisterFastWalkHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_FASTWALK, std::move(handler));
}

void SdkEventBridgeClass::RegisterSwitchMoveModeHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHMOVEMODE, std::move(handler));
}

void SdkEventBridgeClass::RegisterSpeedLevelHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::go2::ROBOT_SPORT_API_ID_SPEEDLEVEL, std::move(handler));
}

void SdkEventBridgeClass::RegisterSwitchGaitHandler(SportEventHandler handler)
{
    RegisterSportEventHandler(unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHGAIT, std::move(handler));
}

bool SdkEventBridgeClass::IsSuccess(int32_t statusCode)
{
    return statusCode == SPORT_STATUS_SUCCESS;
}

bool SdkEventBridgeClass::RequiresPostureActionDelay(int32_t apiId)
{
    switch (apiId)
    {
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND:
        return true;
    default:
        return false;
    }
}

bool SdkEventBridgeClass::IsTrackedSportApi(int32_t apiId)
{
    switch (apiId)
    {
    case unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_BALANCESTAND:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_FREEWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_FREEWALK:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_CLASSICWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_CLASSICWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_VISIONWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_FASTWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHMOVEMODE:
    case unitree::robot::go2::ROBOT_SPORT_API_ID_SPEEDLEVEL:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHGAIT:
        return true;
    default:
        return false;
    }
}

std::string SdkEventBridgeClass::BuildInterceptResponseData(
    int32_t apiId,
    const unitree::robot::Request& request,
    int32_t& statusCode)
{
    statusCode = SPORT_STATUS_SUCCESS;

    switch (apiId)
    {
    case unitree::robot::ROBOT_API_ID_INTERNAL_API_VERSION:
        return unitree::robot::go2::ROBOT_SPORT_API_VERSION;
    case unitree::robot::ROBOT_API_ID_INTERNAL_API_NOOP:
        return {};
    case unitree::robot::ROBOT_API_ID_LEASE_APPLY:
        return R"({"id":1,"term":1000000})";
    case unitree::robot::ROBOT_API_ID_LEASE_RENEWAL:
        return {};
    default:
        (void)request;
        return {};
    }
}

unitree::robot::Response SdkEventBridgeClass::BuildInterceptResponse(
    const unitree::robot::Request& request,
    int32_t statusCode,
    const std::string& data)
{
    unitree::robot::Response response;
    unitree::robot::ResponseHeader header;
    header.identity(request.header().identity());

    unitree::robot::ResponseStatus status;
    status.code(statusCode);
    header.status(status);

    response.header(header);
    response.data(data);
    return response;
}

void SdkEventBridgeClass::HandleInterceptedRequest(const unitree::robot::RequestPtr& requestPtr)
{
    const unitree::robot::Request& request = *requestPtr;
    const int32_t apiId = static_cast<int32_t>(request.header().identity().api_id());
    const auto startedAt = std::chrono::steady_clock::now();

    DispatchRequest(apiId, request);

    int32_t statusCode = SPORT_STATUS_SUCCESS;
    const std::string responseData = BuildInterceptResponseData(apiId, request, statusCode);
    const bool deferSuccessEvent =
        IsTrackedSportApi(apiId) &&
        RequiresPostureActionDelay(apiId) &&
        statusCode == SPORT_STATUS_SUCCESS;

    if (IsTrackedSportApi(apiId) && !deferSuccessEvent)
    {
        SportEventResult result;
        result.apiId = apiId;
        result.requestId = request.header().identity().id();
        result.statusCode = statusCode;
        result.parameter = request.parameter();
        result.data = responseData;
        result.responseLatencyMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startedAt).count();
        DispatchEventResult(result);
    }

    if (deferSuccessEvent)
    {
        std::this_thread::sleep_for(POSTURE_ACTION_RESPONSE_DELAY);
    }

    const unitree::robot::Response response = BuildInterceptResponse(request, statusCode, responseData);

    if (deferSuccessEvent)
    {
        SportEventResult result;
        result.apiId = apiId;
        result.requestId = request.header().identity().id();
        result.statusCode = statusCode;
        result.parameter = request.parameter();
        result.data = responseData;
        result.responseLatencyMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startedAt).count();
        DispatchEventResult(result);
    }

    if (mSportServerStub)
    {
        mSportServerStub->Send(response);
    }

    DispatchResponse(apiId, response);
}

void SdkEventBridgeClass::DispatchRequest(int32_t apiId, const unitree::robot::Request& request)
{
    const auto specificHandler = mRequestHandlers.find(apiId);
    if (specificHandler != mRequestHandlers.end() && specificHandler->second)
    {
        specificHandler->second(apiId, request);
    }

    if (mDefaultRequestHandler)
    {
        mDefaultRequestHandler(apiId, request);
    }
}

void SdkEventBridgeClass::DispatchResponse(int32_t apiId, const unitree::robot::Response& response)
{
    const auto specificHandler = mResponseHandlers.find(apiId);
    if (specificHandler != mResponseHandlers.end() && specificHandler->second)
    {
        specificHandler->second(apiId, response);
    }

    if (mDefaultResponseHandler)
    {
        mDefaultResponseHandler(apiId, response);
    }
}

void SdkEventBridgeClass::DispatchEventResult(const SportEventResult& result)
{
    const auto specificHandler = mEventHandlers.find(result.apiId);
    if (specificHandler != mEventHandlers.end() && specificHandler->second)
    {
        specificHandler->second(result);
    }

    if (mDefaultEventHandler)
    {
        mDefaultEventHandler(result);
    }
}

std::string SdkEventBridgeClass::SportApiIdToString(int32_t apiId)
{
    switch (apiId)
    {
    case unitree::robot::go2::ROBOT_SPORT_API_ID_DAMP:
        return "DAMP";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_MOVE:
        return "MOVE";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_BALANCESTAND:
        return "BALANCESTAND";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STOPMOVE:
        return "STOPMOVE";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDUP:
        return "STANDUP";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_STANDDOWN:
        return "STANDDOWN";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_RECOVERYSTAND:
        return "RECOVERYSTAND";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_FREEWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_FREEWALK:
        return "FREEWALK";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_CLASSICWALK:
    case unitree::robot::b2::ROBOT_SPORT_API_ID_CLASSICWALK:
        return "CLASSICWALK";
    case unitree::robot::b2::ROBOT_SPORT_API_ID_VISIONWALK:
        return "VISIONWALK";
    case unitree::robot::b2::ROBOT_SPORT_API_ID_FASTWALK:
        return "FASTWALK";
    case unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHMOVEMODE:
        return "SWITCHMOVEMODE";
    case unitree::robot::go2::ROBOT_SPORT_API_ID_SPEEDLEVEL:
        return "SPEEDLEVEL";
    case unitree::robot::b2::ROBOT_SPORT_API_ID_SWITCHGAIT:
        return "SWITCHGAIT";
    default:
        return "UNKNOWN(" + std::to_string(apiId) + ")";
    }
}

std::string SdkEventBridgeClass::StatusCodeToString(int32_t statusCode)
{
    if (IsSuccess(statusCode))
    {
        return unitree::robot::UT_ROBOT_OK_DESC;
    }

    switch (statusCode)
    {
    case unitree::robot::UT_ROBOT_ERR_UNKNOWN:
        return unitree::robot::UT_ROBOT_ERR_UNKNOWN_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_SEND:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_SEND_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_API_NOT_REG:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_API_NOT_REG_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_API_TIMEOUT:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_API_TIMEOUT_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_API_NOT_MATCH:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_API_NOT_MATCH_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_API_DATA:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_API_DATA_DESC;
    case unitree::robot::UT_ROBOT_ERR_CLIENT_LEASE_INVALID:
        return unitree::robot::UT_ROBOT_ERR_CLIENT_LEASE_INVALID_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_SEND:
        return unitree::robot::UT_ROBOT_ERR_SERVER_SEND_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_INTERNAL:
        return unitree::robot::UT_ROBOT_ERR_SERVER_INTERNAL_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_API_NOT_IMPL:
        return unitree::robot::UT_ROBOT_ERR_SERVER_API_NOT_IMPL_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_API_PARAMETER:
        return unitree::robot::UT_ROBOT_ERR_SERVER_API_PARAMETER_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_DENIED:
        return unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_DENIED_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_NOT_EXIST:
        return unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_NOT_EXIST_DESC;
    case unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_EXIST:
        return unitree::robot::UT_ROBOT_ERR_SERVER_LEASE_EXIST_DESC;
    case unitree::robot::go2::UT_ROBOT_SPORT_ERR_CLIENT_POINT_PATH:
        return unitree::robot::go2::UT_ROBOT_SPORT_ERR_CLIENT_POINT_PATH_DESC;
    case unitree::robot::go2::UT_ROBOT_SPORT_ERR_SERVER_OVERTIME:
        return unitree::robot::go2::UT_ROBOT_SPORT_ERR_SERVER_OVERTIME_DESC;
    case unitree::robot::go2::UT_ROBOT_SPORT_ERR_SERVER_NOT_INIT:
        return unitree::robot::go2::UT_ROBOT_SPORT_ERR_SERVER_NOT_INIT_DESC;
    default:
        return "Error(" + std::to_string(statusCode) + ")";
    }
}

}  // namespace sdk_event_bridge
