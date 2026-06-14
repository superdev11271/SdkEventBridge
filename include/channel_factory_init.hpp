#ifndef CHANNEL_FACTORY_INIT_HPP
#define CHANNEL_FACTORY_INIT_HPP

#include <cstdint>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>

namespace sdk_event_bridge
{

inline void InitUnitreeChannelFactoryOnce(int32_t domainId, const std::string& networkInterface)
{
    static bool initialized = false;
    if (initialized)
    {
        return;
    }

    unitree::robot::ChannelFactory::Instance()->Init(domainId, networkInterface);
    initialized = true;
}

}  // namespace sdk_event_bridge

#endif  // CHANNEL_FACTORY_INIT_HPP
