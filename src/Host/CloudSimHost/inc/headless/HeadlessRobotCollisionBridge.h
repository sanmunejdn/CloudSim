#ifndef CLOUDSIMHOST_HEADLESSROBOTCOLLISIONBRIDGE_H
#define CLOUDSIMHOST_HEADLESSROBOTCOLLISIONBRIDGE_H

/// @file HeadlessRobotCollisionBridge.h
/// @brief Web/Headless：碰撞检测设置与规划（桩）

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessRobotCollisionBridge
{
public:
	explicit HeadlessRobotCollisionBridge(DocumentHost& host);

	HeadlessRobotCollisionBridge(const HeadlessRobotCollisionBridge&) = delete;
	HeadlessRobotCollisionBridge& operator=(const HeadlessRobotCollisionBridge&) = delete;

	QJsonObject getSettings(const QJsonObject& body);
	QJsonObject putSettings(const QJsonObject& body);
	QJsonObject plan(const QJsonObject& body);
	QJsonObject confirm(const QJsonObject& body);

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
