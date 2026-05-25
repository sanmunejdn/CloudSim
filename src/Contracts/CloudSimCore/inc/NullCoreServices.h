#pragma once

#include "cloudsim_core_global.h"

#include <memory>

namespace cloudsim::core {

class IDataService;
class IRobotService;
class IRenderViewFactory;

/// 空实现桩
CLOUDSIM_CORE_EXPORT std::unique_ptr<IDataService> makeNullDataService();
CLOUDSIM_CORE_EXPORT std::unique_ptr<IRobotService> makeNullRobotService();
CLOUDSIM_CORE_EXPORT std::unique_ptr<IRenderViewFactory> makeNullRenderViewFactory();

} // namespace cloudsim::core
