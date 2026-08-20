#ifndef CLOUDSIMBOOTSTRAP_CLOUDSIMBOOTSTRAP_H
#define CLOUDSIMBOOTSTRAP_CLOUDSIMBOOTSTRAP_H

/// @file CloudSimBootstrap.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 组合根 API（实现在 CloudSimHost.dll）

#include "cloudsim_host_global.h"

#include <memory>

namespace cloudsim::core
{
class ICloudSimContext;
}

/// 组合根 API（实现在 CloudSimHost.dll）
CLOUDSIM_HOST_EXPORT std::unique_ptr<cloudsim::core::ICloudSimContext> cloudsimCreateApplicationContext();

/// Web 进程旁路组合根：Null 渲染工厂 + Headless 文档；桌面勿调用
CLOUDSIM_HOST_EXPORT std::unique_ptr<cloudsim::core::ICloudSimContext> cloudsimCreateHeadlessApplicationContext();

CLOUDSIM_HOST_EXPORT void cloudsimSetApplicationContext(std::unique_ptr<cloudsim::core::ICloudSimContext> context);
CLOUDSIM_HOST_EXPORT cloudsim::core::ICloudSimContext* cloudsimApplicationContext();

#endif // CLOUDSIMBOOTSTRAP_CLOUDSIMBOOTSTRAP_H
