#ifndef CLOUDSIMHOST_HEADLESSLABELINGBRIDGE_H
#define CLOUDSIMHOST_HEADLESSLABELINGBRIDGE_H

/// @file HeadlessLabelingBridge.h
/// @brief Web/Headless：标注任务列表

#include "cloudsim_host_global.h"

#include <QJsonArray>
#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessLabelingBridge
{
public:
	explicit HeadlessLabelingBridge(DocumentHost& host);

	HeadlessLabelingBridge(const HeadlessLabelingBridge&) = delete;
	HeadlessLabelingBridge& operator=(const HeadlessLabelingBridge&) = delete;

	QJsonObject tasksJson() const;

	void loadSidecarFromProject(const QJsonObject& projectRoot);
	void mergeSidecarIntoProject(QJsonObject& projectRoot) const;

private:
	DocumentHost& m_host;
	QJsonArray m_annotations;
};

} // namespace cloudsim::host

#endif
