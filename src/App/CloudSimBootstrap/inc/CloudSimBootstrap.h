#pragma once

#include "cloudsim_host_global.h"

#include <memory>

namespace cloudsim::core {
class ICloudSimContext;
}

/// 组合根 API（实现在 CloudSimHost.dll）。
CLOUDSIM_HOST_EXPORT std::unique_ptr<cloudsim::core::ICloudSimContext> cloudsimCreateApplicationContext();

CLOUDSIM_HOST_EXPORT void cloudsimSetApplicationContext(std::unique_ptr<cloudsim::core::ICloudSimContext> context);
CLOUDSIM_HOST_EXPORT cloudsim::core::ICloudSimContext* cloudsimApplicationContext();
