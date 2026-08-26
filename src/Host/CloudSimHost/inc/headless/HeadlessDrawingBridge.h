#ifndef CLOUDSIMHOST_HEADLESSDRAWINGBRIDGE_H
#define CLOUDSIMHOST_HEADLESSDRAWINGBRIDGE_H

/// @file HeadlessDrawingBridge.h
/// @brief Web/Headless：工程图导出

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessDrawingBridge
{
public:
	explicit HeadlessDrawingBridge(DocumentHost& host);

	HeadlessDrawingBridge(const HeadlessDrawingBridge&) = delete;
	HeadlessDrawingBridge& operator=(const HeadlessDrawingBridge&) = delete;

	QJsonObject exportDrawing(const QJsonObject& body);

	void loadSidecarFromProject(const QJsonObject& projectRoot);
	void mergeSidecarIntoProject(QJsonObject& projectRoot) const;

private:
	DocumentHost& m_host;
	QJsonObject m_engineeringDrawing;
};

} // namespace cloudsim::host

#endif
