/// @file HeadlessProcessFlowBridge.cpp

#include "headless/HeadlessProcessFlowBridge.h"

#include "BackendTypeIds.h"
#include "DocumentHost.h"

#include "sim/DesEngine.h"

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}
} // namespace

HeadlessProcessFlowBridge::HeadlessProcessFlowBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessProcessFlowBridge::getGraph() const
{
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("graph"), m_processFlow);
	return o;
}

QJsonObject HeadlessProcessFlowBridge::putGraph(const QJsonObject& body)
{
	const QJsonObject graph = body.value(QStringLiteral("graph")).toObject();
	if (graph.isEmpty() && body.contains(QStringLiteral("nodes")))
		m_processFlow = body;
	else if (!graph.isEmpty())
		m_processFlow = graph;
	else
		return fail(QStringLiteral("graph object required."));

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	return o;
}

QJsonObject HeadlessProcessFlowBridge::runSim(const QJsonObject& body)
{
	(void)body;
	(void)m_host;
	(void)sizeof(DesEngine);

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	o.insert(QStringLiteral("note"), QStringLiteral("DES simulation stub"));
	return o;
}

void HeadlessProcessFlowBridge::loadSidecarFromProject(const QJsonObject& projectRoot)
{
	m_processFlow = projectRoot.value(QLatin1String(backend_type::kProjectKeyProcessFlow)).toObject();
}

void HeadlessProcessFlowBridge::mergeSidecarIntoProject(QJsonObject& projectRoot) const
{
	if (!m_processFlow.isEmpty())
		projectRoot.insert(QLatin1String(backend_type::kProjectKeyProcessFlow), m_processFlow);
}

} // namespace cloudsim::host
