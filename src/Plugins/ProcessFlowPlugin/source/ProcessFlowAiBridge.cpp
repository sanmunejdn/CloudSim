/// @file ProcessFlowAiBridge.cpp
/// @brief 工艺流程 AI 桥接实现

#include "ProcessFlowAiBridge.h"

#include "ProcessFlowCanvasWidget.h"
#include "ProcessFlowJobSetPanel.h"
#include "ProcessFlowPageWidget.h"
#include "ProcessFlowPlugin.h"
#include "ProcessFlowReportPanel.h"
#include "ProcessFlowSimSideWidget.h"
#include "sim/DesEngine.h"
#include "sim/DispatchPolicies.h"
#include "sim/IStationExecutor.h"
#include "sim/SimModelBuilder.h"
#include "sim/SimRunConfig.h"
#include "sim/SimStatistics.h"

#include <QJsonArray>
#include <QJsonObject>

#include <memory>

ProcessFlowAiBridge::ProcessFlowAiBridge(ProcessFlowPlugin* plugin) : m_plugin(plugin) {}

bool ProcessFlowAiBridge::ensureEntered(QString* outError)
{
	if (!m_plugin)
	{
		if (outError)
			*outError = QStringLiteral("工艺流程插件不可用。");
		return false;
	}
	return m_plugin->ensureProcessFlowForAi(outError);
}

bool ProcessFlowAiBridge::applyFlowJson(const QJsonObject& flow, bool doAutoLayout, QString* outError)
{
	if (!ensureEntered(outError))
		return false;

	SimRunConfig cfg;
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(flow, cfg);
	if (!built.ok)
	{
		if (outError)
			*outError = built.error.isEmpty() ? QStringLiteral("流程图校验失败。") : built.error;
		return false;
	}

	ProcessFlowCanvasWidget* canvas = m_plugin->activeCanvasForAi();
	if (!canvas)
	{
		if (outError)
			*outError = QStringLiteral("无活动流程画布。");
		return false;
	}

	if (!canvas->fromJson(flow))
	{
		if (outError)
			*outError = QStringLiteral("写入流程图失败。");
		return false;
	}
	if (doAutoLayout)
		canvas->autoLayout();

	m_plugin->syncJobSetPanelFromCanvas();
	return true;
}

bool ProcessFlowAiBridge::runSimSync(const QJsonObject& config, QJsonObject* outStats, QString* outError)
{
	if (!ensureEntered(outError))
		return false;

	ProcessFlowCanvasWidget* canvas = m_plugin->activeCanvasForAi();
	if (!canvas)
	{
		if (outError)
			*outError = QStringLiteral("无活动流程画布。");
		return false;
	}

	SimRunConfig cfg;
	if (config.contains(QStringLiteral("horizonSec")))
		cfg.horizonSec = config.value(QStringLiteral("horizonSec")).toDouble(cfg.horizonSec);
	if (config.contains(QStringLiteral("policy")))
		cfg.policy = config.value(QStringLiteral("policy")).toString(cfg.policy).trimmed().toLower();
	if (cfg.policy.isEmpty())
		cfg.policy = QStringLiteral("fifo");

	const QJsonObject flow = canvas->toJson();
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(flow, cfg);
	if (!built.ok)
	{
		if (outError)
			*outError = built.error;
		return false;
	}

	DesEngine engine;
	engine.setDispatchPolicy(createDispatchPolicy(cfg.policy));
	engine.setStationExecutor(std::make_unique<NullStationExecutor>());
	const SimStatistics stats = engine.run(built.plant, built.jobSet, built.interarrivalSec, cfg, nullptr);

	m_plugin->applyAiSimStatistics(stats);

	if (outStats)
		*outStats = stats.toJson();
	return true;
}

bool ProcessFlowAiBridge::compareSync(const QJsonObject& config, QJsonArray* outRows, QString* outError)
{
	if (!ensureEntered(outError))
		return false;

	ProcessFlowCanvasWidget* canvas = m_plugin->activeCanvasForAi();
	if (!canvas)
	{
		if (outError)
			*outError = QStringLiteral("无活动流程画布。");
		return false;
	}

	SimRunConfig cfg;
	if (config.contains(QStringLiteral("horizonSec")))
		cfg.horizonSec = config.value(QStringLiteral("horizonSec")).toDouble(cfg.horizonSec);

	const QJsonObject flow = canvas->toJson();
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(flow, cfg);
	if (!built.ok)
	{
		if (outError)
			*outError = built.error;
		return false;
	}

	QVector<PolicyCompareRow> rows;
	for (const QString& policyName : allDispatchPolicyNames())
	{
		DesEngine engine;
		engine.setDispatchPolicy(createDispatchPolicy(policyName));
		engine.setStationExecutor(std::make_unique<NullStationExecutor>());
		SimRunConfig c = cfg;
		c.policy = policyName;
		const SimStatistics st = engine.run(built.plant, built.jobSet, built.interarrivalSec, c, nullptr);
		PolicyCompareRow row;
		row.policy = policyName.toUpper();
		row.makespan = st.makespan;
		row.completed = st.completedJobs;
		row.throughput = st.throughputPerHour;
		row.bottleneck = st.bottleneckTitle.isEmpty() ? QString::number(st.bottleneckNodeId) : st.bottleneckTitle;
		rows.append(row);
	}

	m_plugin->applyAiCompareRows(rows);

	if (outRows)
	{
		QJsonArray arr;
		for (const PolicyCompareRow& row : rows)
		{
			QJsonObject o;
			o.insert(QStringLiteral("policy"), row.policy);
			o.insert(QStringLiteral("makespan"), row.makespan);
			o.insert(QStringLiteral("completed"), row.completed);
			o.insert(QStringLiteral("throughput"), row.throughput);
			o.insert(QStringLiteral("bottleneck"), row.bottleneck);
			arr.append(o);
		}
		*outRows = arr;
	}
	return true;
}

QJsonObject ProcessFlowAiBridge::exportFlowJson() const
{
	if (!m_plugin)
		return {};
	ProcessFlowCanvasWidget* canvas = m_plugin->activeCanvasForAi();
	return canvas ? canvas->toJson() : QJsonObject{};
}
