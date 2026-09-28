#ifndef DATA_BACKENDMANAGERDATASERVICE_H
#define DATA_BACKENDMANAGERDATASERVICE_H

/// @file BackendManagerDataService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Data 层 IDataService：BackendDataManager 直驱（无视觉/Follow 求解副作用）

#include "data_global.h"

#include <memory>

namespace cloudsim::core
{
class IDataService;
}

/// cloudsimCreateDataService 的实现真源；视觉分支/Follow 求解等 Host 能力降级为告警+空转
class BackendDataManager;

/// @param managerOverride 非空时绑定文档级 BackendDataManager；空则回退 BackendDataManager::instance()
DATA_EXPORT std::unique_ptr<cloudsim::core::IDataService> makeBackendManagerDataService(
	BackendDataManager* managerOverride = nullptr);

#endif // DATA_BACKENDMANAGERDATASERVICE_H
