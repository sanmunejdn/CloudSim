#ifndef CLOUDSIMWEBGATEWAY_DEVICECATALOGSCAN_H
#define CLOUDSIMWEBGATEWAY_DEVICECATALOGSCAN_H

/// @file DeviceCatalogScan.h
/// @brief 扫描 resource/models 下 URDF 设备包（对齐桌面 DevicePageWidget，不链 RobotWidget）

#include "cloudsim_web_gateway_global.h"

#include <QJsonObject>
#include <QString>

namespace cloudsim::web
{
/// modelsRoot 一般为 applicationDir/resource/models
CLOUDSIM_WEB_GATEWAY_API QJsonObject scanDeviceCatalog(const QString& modelsRoot);
} // namespace cloudsim::web

#endif // CLOUDSIMWEBGATEWAY_DEVICECATALOGSCAN_H
