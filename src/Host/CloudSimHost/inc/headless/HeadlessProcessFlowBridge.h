#ifndef CLOUDSIMHOST_HEADLESSPROCESSFLOWBRIDGE_H
#define CLOUDSIMHOST_HEADLESSPROCESSFLOWBRIDGE_H

/// @file HeadlessProcessFlowBridge.h
/// @brief Web/Headless：工艺流程图与 DES 仿真

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessProcessFlowBridge
{
public:
	explicit HeadlessProcessFlowBridge(DocumentHost& host);

	HeadlessProcessFlowBridge(const HeadlessProcessFlowBridge&) = delete;
	HeadlessProcessFlowBridge& operator=(const HeadlessProcessFlowBridge&) = delete;

	QJsonObject getGraph() const;
	QJsonObject putGraph(const QJsonObject& body);
	QJsonObject runSim(const QJsonObject& body);

	void loadSidecarFromProject(const QJsonObject& projectRoot);
	void mergeSidecarIntoProject(QJsonObject& projectRoot) const;

private:
	DocumentHost& m_host;
	QJsonObject m_processFlow;
};

} // namespace cloudsim::host

#endif
