#include "motion_switcher_bridge.hpp"

#include "channel_factory_init.hpp"
#include <unitree/common/json/jsonize.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>

#include <unitree/robot/b2/motion_switcher/motion_switcher_error.hpp>
#include <unitree/robot/internal/internal_api.hpp>
#include <unitree/robot/internal/internal_error.hpp>

namespace sdk_event_bridge
{

MotionSwitcherBridgeClass::MotionSwitcherBridgeClass(
    int32_t domainId,
    const std::string& networkInterface)
    : mDomainId(domainId),
      mNetworkInterface(networkInterface),
      mInitialized(false),
      mStarted(false),
      mForm("0"),
      mModeName(),
      mSilent(false),
      mRobotPosture(RobotPosture::StandDown)
{
}

MotionSwitcherBridgeClass::~MotionSwitcherBridgeClass()
{
    Stop();
}

void MotionSwitcherBridgeClass::Init()
{
    if (mInitialized)
    {
        return;
    }

    InitUnitreeChannelFactoryOnce(mDomainId, mNetworkInterface);
    mInitialized = true;
}

void MotionSwitcherBridgeClass::Start()
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

void MotionSwitcherBridgeClass::Stop()
{
    StopIntercept();
    mStarted = false;
}

void MotionSwitcherBridgeClass::StartIntercept()
{
    mResponsePublisher = std::make_unique<unitree::robot::ChannelPublisher<unitree::robot::Response>>(
        MOTION_SWITCHER_RESPONSE_TOPIC);
    mResponsePublisher->InitChannel();

    mRequestSubscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree::robot::Request>>(
        MOTION_SWITCHER_REQUEST_TOPIC,
        [this](const void* message) {
            const auto* request = static_cast<const unitree::robot::Request*>(message);
            HandleInterceptedRequest(*request);
        },
        10);
    mRequestSubscriber->InitChannel();
}

void MotionSwitcherBridgeClass::StopIntercept()
{
    if (mRequestSubscriber)
    {
        mRequestSubscriber->CloseChannel();
        mRequestSubscriber.reset();
    }

    if (mResponsePublisher)
    {
        mResponsePublisher->CloseChannel();
        mResponsePublisher.reset();
    }
}

void MotionSwitcherBridgeClass::SetRequestHandler(MotionSwitcherRequestHandler handler)
{
    mDefaultRequestHandler = std::move(handler);
}

void MotionSwitcherBridgeClass::SetResponseHandler(MotionSwitcherResponseHandler handler)
{
    mDefaultResponseHandler = std::move(handler);
}

void MotionSwitcherBridgeClass::SetEventHandler(MotionSwitcherEventHandler handler)
{
    mDefaultEventHandler = std::move(handler);
}

void MotionSwitcherBridgeClass::RegisterRequestHandler(
    int32_t apiId,
    MotionSwitcherRequestHandler handler)
{
    mRequestHandlers[apiId] = std::move(handler);
}

void MotionSwitcherBridgeClass::RegisterResponseHandler(
    int32_t apiId,
    MotionSwitcherResponseHandler handler)
{
    mResponseHandlers[apiId] = std::move(handler);
}

void MotionSwitcherBridgeClass::RegisterEventHandler(
    int32_t apiId,
    MotionSwitcherEventHandler handler)
{
    mEventHandlers[apiId] = std::move(handler);
}

void MotionSwitcherBridgeClass::RegisterCheckModeHandler(MotionSwitcherEventHandler handler)
{
    RegisterEventHandler(unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE, std::move(handler));
}

void MotionSwitcherBridgeClass::RegisterSelectModeHandler(MotionSwitcherEventHandler handler)
{
    RegisterEventHandler(unitree::robot::b2::MOTION_SWITCHER_API_ID_SELECT_MODE, std::move(handler));
}

void MotionSwitcherBridgeClass::RegisterReleaseModeHandler(MotionSwitcherEventHandler handler)
{
    RegisterEventHandler(unitree::robot::b2::MOTION_SWITCHER_API_ID_RELEASE_MODE, std::move(handler));
}

void MotionSwitcherBridgeClass::RegisterSetSilentHandler(MotionSwitcherEventHandler handler)
{
    RegisterEventHandler(unitree::robot::b2::MOTION_SWITCHER_API_ID_SET_SILENT, std::move(handler));
}

void MotionSwitcherBridgeClass::RegisterGetSilentHandler(MotionSwitcherEventHandler handler)
{
    RegisterEventHandler(unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT, std::move(handler));
}

std::string MotionSwitcherBridgeClass::GetForm() const
{
    return mForm;
}

std::string MotionSwitcherBridgeClass::GetModeName() const
{
    return mModeName;
}

bool MotionSwitcherBridgeClass::GetSilent() const
{
    return mSilent;
}

void MotionSwitcherBridgeClass::SetRobotPosture(RobotPosture posture)
{
    if (mRobotPosture == posture)
    {
        return;
    }

    mRobotPosture = posture;
    std::cout << "[robot_posture] " << RobotPostureToString(posture) << std::endl;
}

RobotPosture MotionSwitcherBridgeClass::GetRobotPosture() const
{
    return mRobotPosture;
}

bool MotionSwitcherBridgeClass::CanChangeMotionMode() const
{
    return mRobotPosture == RobotPosture::StandDown;
}

const char* MotionSwitcherBridgeClass::RobotPostureToString(RobotPosture posture)
{
    switch (posture)
    {
    case RobotPosture::StandDown:
        return "stand down";
    case RobotPosture::StandUp:
        return "stand up";
    case RobotPosture::Walking:
        return "walking";
    default:
        return "unknown";
    }
}

bool MotionSwitcherBridgeClass::IsTrackedApi(int32_t apiId)
{
    switch (apiId)
    {
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE:
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SELECT_MODE:
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_RELEASE_MODE:
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SET_SILENT:
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT:
        return true;
    default:
        return false;
    }
}

std::string MotionSwitcherBridgeClass::ApiIdToString(int32_t apiId)
{
    switch (apiId)
    {
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE:
        return "CHECK_MODE";
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SELECT_MODE:
        return "SELECT_MODE";
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_RELEASE_MODE:
        return "RELEASE_MODE";
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SET_SILENT:
        return "SET_SILENT";
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT:
        return "GET_SILENT";
    default:
        return "UNKNOWN(" + std::to_string(apiId) + ")";
    }
}

std::string MotionSwitcherBridgeClass::BuildInterceptResponseData(
    int32_t apiId,
    const unitree::robot::Request& request,
    int32_t& statusCode)
{
    statusCode = SPORT_STATUS_SUCCESS;

    switch (apiId)
    {
    case unitree::robot::ROBOT_API_ID_INTERNAL_API_VERSION:
        return unitree::robot::b2::MOTION_SWITCHER_API_VERSION;
    case unitree::robot::ROBOT_API_ID_INTERNAL_API_NOOP:
        return {};
    case unitree::robot::ROBOT_API_ID_LEASE_APPLY:
        return R"({"id":1,"term":1000000})";
    case unitree::robot::ROBOT_API_ID_LEASE_RENEWAL:
        return {};
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_CHECK_MODE:
    {
        unitree::robot::b2::JsonizeModeName modeData;
        modeData.form = mForm;
        modeData.name = mModeName;
        return unitree::common::ToJsonString(modeData);
    }
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SELECT_MODE:
    {
        if (!CanChangeMotionMode())
        {
            statusCode = unitree::robot::b2::UT_SWITCH_ERR_BUSY;
            std::cout << "[motion_switcher] SELECT_MODE rejected while "
                      << RobotPostureToString(mRobotPosture) << std::endl;
            return {};
        }

        unitree::robot::b2::JsonizeModeName modeData;
        try
        {
            unitree::common::FromJsonString(request.parameter(), modeData);
        }
        catch (const std::exception&)
        {
            statusCode = unitree::robot::UT_ROBOT_ERR_SERVER_API_PARAMETER;
            return {};
        }

        if (modeData.name.empty())
        {
            statusCode = unitree::robot::UT_ROBOT_ERR_SERVER_API_PARAMETER;
            return {};
        }

        mModeName = modeData.name;
        if (!modeData.form.empty())
        {
            mForm = modeData.form;
        }
        return {};
    }
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_RELEASE_MODE:
        if (!CanChangeMotionMode())
        {
            statusCode = unitree::robot::b2::UT_SWITCH_ERR_BUSY;
            std::cout << "[motion_switcher] RELEASE_MODE rejected while "
                      << RobotPostureToString(mRobotPosture) << std::endl;
            return {};
        }

        mModeName.clear();
        return {};
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_SET_SILENT:
    {
        unitree::robot::b2::JsonizeSilent silentData;
        try
        {
            unitree::common::FromJsonString(request.parameter(), silentData);
        }
        catch (const std::exception&)
        {
            statusCode = unitree::robot::UT_ROBOT_ERR_SERVER_API_PARAMETER;
            return {};
        }

        mSilent = silentData.silent;
        return {};
    }
    case unitree::robot::b2::MOTION_SWITCHER_API_ID_GET_SILENT:
    {
        unitree::robot::b2::JsonizeSilent silentData;
        silentData.silent = mSilent;
        return unitree::common::ToJsonString(silentData);
    }
    default:
        (void)request;
        return {};
    }
}

unitree::robot::Response MotionSwitcherBridgeClass::BuildInterceptResponse(
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

void MotionSwitcherBridgeClass::HandleInterceptedRequest(
    const unitree::robot::Request& request)
{
    const int32_t apiId = static_cast<int32_t>(request.header().identity().api_id());
    const auto startedAt = std::chrono::steady_clock::now();

    DispatchRequest(apiId, request);

    int32_t statusCode = SPORT_STATUS_SUCCESS;
    const std::string responseData = BuildInterceptResponseData(apiId, request, statusCode);
    const unitree::robot::Response response =
        BuildInterceptResponse(request, statusCode, responseData);

    if (IsTrackedApi(apiId))
    {
        MotionSwitcherEventResult result;
        result.apiId = apiId;
        result.requestId = request.header().identity().id();
        result.statusCode = statusCode;
        result.parameter = request.parameter();
        result.data = responseData;
        result.responseLatencyMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startedAt).count();
        DispatchEventResult(result);
    }

    if (mResponsePublisher)
    {
        mResponsePublisher->Write(response);
    }

    DispatchResponse(apiId, response);
}

void MotionSwitcherBridgeClass::DispatchRequest(
    int32_t apiId,
    const unitree::robot::Request& request)
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

void MotionSwitcherBridgeClass::DispatchResponse(
    int32_t apiId,
    const unitree::robot::Response& response)
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

void MotionSwitcherBridgeClass::DispatchEventResult(const MotionSwitcherEventResult& result)
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

}  // namespace sdk_event_bridge
