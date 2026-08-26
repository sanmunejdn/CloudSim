/// @file HeadlessLabelingBridge.cpp

#include "headless/HeadlessLabelingBridge.h"

#include "DocumentHost.h"

namespace cloudsim::host
{
HeadlessLabelingBridge::HeadlessLabelingBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessLabelingBridge::tasksJson() const
{
	(void)m_host;

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("tasks"), m_annotations);
	o.insert(QStringLiteral("count"), m_annotations.size());
	return o;
}

void HeadlessLabelingBridge::loadSidecarFromProject(const QJsonObject& projectRoot)
{
	m_annotations = projectRoot.value(QStringLiteral("annotations")).toArray();
}

void HeadlessLabelingBridge::mergeSidecarIntoProject(QJsonObject& projectRoot) const
{
	if (!m_annotations.isEmpty())
		projectRoot.insert(QStringLiteral("annotations"), m_annotations);
}

} // namespace cloudsim::host
