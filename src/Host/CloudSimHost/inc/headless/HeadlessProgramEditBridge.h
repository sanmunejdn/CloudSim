#ifndef CLOUDSIMHOST_HEADLESSPROGRAMEDITBRIDGE_H
#define CLOUDSIMHOST_HEADLESSPROGRAMEDITBRIDGE_H

/// @file HeadlessProgramEditBridge.h
/// @brief Web/Headless：程序编辑 undo/redo/切换/分组

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessProgramEditBridge
{
public:
	explicit HeadlessProgramEditBridge(DocumentHost& host);

	HeadlessProgramEditBridge(const HeadlessProgramEditBridge&) = delete;
	HeadlessProgramEditBridge& operator=(const HeadlessProgramEditBridge&) = delete;

	QJsonObject undo(const QJsonObject& body);
	QJsonObject redo(const QJsonObject& body);
	QJsonObject switchProgram(const QJsonObject& body);
	QJsonObject groupCrud(const QJsonObject& body);

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
