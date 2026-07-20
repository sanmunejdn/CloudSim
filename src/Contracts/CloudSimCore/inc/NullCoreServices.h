#ifndef CLOUDSIMCORE_NULLCORESERVICES_H
#define CLOUDSIMCORE_NULLCORESERVICES_H

/// @file NullCoreServices.h
/// @brief 空实现桩

#include "cloudsim_core_global.h"

#include <memory>

namespace cloudsim::core
{
class IDataService;
class IRobotService;
class IRenderViewFactory;

/// 空实现桩
CLOUDSIM_CORE_EXPORT std::unique_ptr<IDataService> makeNullDataService();
CLOUDSIM_CORE_EXPORT std::unique_ptr<IRobotService> makeNullRobotService();
CLOUDSIM_CORE_EXPORT std::unique_ptr<IRenderViewFactory> makeNullRenderViewFactory();

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_NULLCORESERVICES_H
