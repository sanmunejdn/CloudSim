#ifndef CLOUDSIMHOST_HEADLESSROBOTEXPORTBRIDGE_H
#define CLOUDSIMHOST_HEADLESSROBOTEXPORTBRIDGE_H

/// @file HeadlessRobotExportBridge.h
/// @brief Web/Headless：机器人程序 Canonical 导出

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessRobotExportBridge
{
public:
	explicit HeadlessRobotExportBridge(DocumentHost& host);

	HeadlessRobotExportBridge(const HeadlessRobotExportBridge&) = delete;
	HeadlessRobotExportBridge& operator=(const HeadlessRobotExportBridge&) = delete;

	QJsonObject exportProgram(const QJsonObject& body);

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
