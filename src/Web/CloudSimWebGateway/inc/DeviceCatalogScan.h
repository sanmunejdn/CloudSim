#ifndef CLOUDSIMWEBGATEWAY_DEVICECATALOGSCAN_H
#define CLOUDSIMWEBGATEWAY_DEVICECATALOGSCAN_H

/// @file DeviceCatalogScan.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
