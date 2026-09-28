#ifndef CLOUDSIMCORE_SERVICEREGISTRY_H
#define CLOUDSIMCORE_SERVICEREGISTRY_H

/// @file ServiceRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 服务注册表（非单例，由 ICloudSimContext 持有）

#include "cloudsim_core_global.h"

#include <memory>
#include <typeindex>
#include <unordered_map>

namespace cloudsim::core
{
/// 服务注册表（非单例，由 ICloudSimContext 持有），消除全局单例
class CLOUDSIM_CORE_EXPORT ServiceRegistry
{
public:
    ServiceRegistry() = default;
    ~ServiceRegistry() = default;
    ServiceRegistry(const ServiceRegistry&) = delete;
    ServiceRegistry& operator=(const ServiceRegistry&) = delete;

    template <typename T>
    void registerService(std::shared_ptr<T> svc)
    {
        m_services[std::type_index(typeid(T))] = std::move(svc);
    }

    template <typename T>
    std::shared_ptr<T> getService() const
    {
        const auto it = m_services.find(std::type_index(typeid(T)));
        if (it == m_services.end())
            return nullptr;
        return std::static_pointer_cast<T>(it->second);
    }

private:
    std::unordered_map<std::type_index, std::shared_ptr<void>> m_services;
};
} // namespace cloudsim::core

#endif // CLOUDSIMCORE_SERVICEREGISTRY_H
