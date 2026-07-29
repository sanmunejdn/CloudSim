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

#include <QColor>
#include <QJsonArray>
#include <QJsonObject>
#include <QPointF>

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

bool ProcessFlowAiBridge::applyFlowPatch(const QJsonArray& ops, QString* outError)
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

	for (const QJsonValue& v : ops)
	{
		const QJsonObject o = v.toObject();
		const QString op = o.value(QStringLiteral("op")).toString().trimmed().toLower();
		if (op == QStringLiteral("setnodeprop"))
		{
			const int id = o.value(QStringLiteral("nodeId")).toInt(-1);
			ProcessFlowNodeProps props;
			if (!canvas->nodeProps(id, &props))
			{
				if (outError)
					*outError = QStringLiteral("节点不存在: %1").arg(id);
				return false;
			}
			const QJsonObject p = o.value(QStringLiteral("props")).toObject();
			if (p.contains(QStringLiteral("kind")))
				props.kind = p.value(QStringLiteral("kind")).toString(props.kind);
			if (p.contains(QStringLiteral("cycleTimeSec")))
				props.cycleTimeSec = p.value(QStringLiteral("cycleTimeSec")).toDouble(props.cycleTimeSec);
			if (p.contains(QStringLiteral("capacityQty")))
				props.capacityQty = p.value(QStringLiteral("capacityQty")).toDouble(props.capacityQty);
			if (p.contains(QStringLiteral("inventoryQty")))
				props.inventoryQty = p.value(QStringLiteral("inventoryQty")).toDouble(props.inventoryQty);
			if (p.contains(QStringLiteral("scrapRate")))
				props.scrapRate = p.value(QStringLiteral("scrapRate")).toDouble(props.scrapRate);
			if (p.contains(QStringLiteral("batchSize")))
				props.batchSize = p.value(QStringLiteral("batchSize")).toDouble(props.batchSize);
			if (p.contains(QStringLiteral("mtbfSec")))
				props.mtbfSec = p.value(QStringLiteral("mtbfSec")).toDouble(props.mtbfSec);
			if (p.contains(QStringLiteral("mttrSec")))
				props.mttrSec = p.value(QStringLiteral("mttrSec")).toDouble(props.mttrSec);
			const QJsonObject binding = p.value(QStringLiteral("binding")).toObject();
			if (!binding.isEmpty())
			{
				props.bindingBackendId = binding.value(QStringLiteral("backendId")).toString(props.bindingBackendId);
				props.bindingProgramId = binding.value(QStringLiteral("programId")).toString(props.bindingProgramId);
			}
			if (!canvas->setNodeProps(id, props))
			{
				if (outError)
					*outError = QStringLiteral("设置节点属性失败。");
				return false;
			}
		}
		else if (op == QStringLiteral("addnode"))
		{
			const QString kind = o.value(QStringLiteral("kind")).toString(QStringLiteral("station"));
			const QString title = o.value(QStringLiteral("title")).toString(ProcessFlowNodeProps::displayNameZh(kind));
			const QString subtitle = o.value(QStringLiteral("subtitle")).toString();
			const double x = o.value(QStringLiteral("x")).toDouble(40.0);
			const double y = o.value(QStringLiteral("y")).toDouble(40.0);
			canvas->addNode(title, subtitle, QColor(QStringLiteral("#2563EB")), QPointF(x, y), kind);
		}
		else if (op == QStringLiteral("removenode"))
		{
			const int id = o.value(QStringLiteral("nodeId")).toInt(-1);
			if (!canvas->removeNodeById(id))
			{
				if (outError)
					*outError = QStringLiteral("删除节点失败: %1").arg(id);
				return false;
			}
		}
		else if (op == QStringLiteral("connect"))
		{
			canvas->addEdge(o.value(QStringLiteral("from")).toInt(), o.value(QStringLiteral("to")).toInt(),
							o.value(QStringLiteral("label")).toString());
		}
		else if (op == QStringLiteral("disconnect"))
		{
			if (!canvas->removeEdge(o.value(QStringLiteral("from")).toInt(), o.value(QStringLiteral("to")).toInt()))
			{
				if (outError)
					*outError = QStringLiteral("删除边失败。");
				return false;
			}
		}
		else
		{
			if (outError)
				*outError = QStringLiteral("未知 op: %1").arg(op);
			return false;
		}
	}
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
	if (config.contains(QStringLiteral("warmupSec")))
		cfg.warmupSec = config.value(QStringLiteral("warmupSec")).toDouble(cfg.warmupSec);
	if (config.contains(QStringLiteral("policy")))
		cfg.policy = config.value(QStringLiteral("policy")).toString(cfg.policy).trimmed().toLower();
	if (config.contains(QStringLiteral("arrivalMode")))
		cfg.arrivalMode = config.value(QStringLiteral("arrivalMode")).toString(cfg.arrivalMode);
	if (cfg.policy.isEmpty())
		cfg.policy = QStringLiteral("fifo");
	const bool openGantt = config.value(QStringLiteral("openGantt")).toBool(false);

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
	if (cfg.executorMode.compare(QStringLiteral("drivePreview"), Qt::CaseInsensitive) == 0)
		engine.setStationExecutor(std::make_unique<PreviewStationExecutor>());
	else
		engine.setStationExecutor(std::make_unique<NullStationExecutor>());
	const SimStatistics stats = engine.run(built.plant, built.jobSet, built.interarrivalSec, cfg, nullptr);

	m_plugin->applyAiSimStatistics(stats);
	if (openGantt && m_plugin)
	{
		// 经报表面板打开甘特
	}

	if (outStats)
	{
		*outStats = stats.toJson();
		if (!stats.buffers.isEmpty())
		{
			const BufferStat& top = stats.buffers.first();
			outStats->insert(QStringLiteral("topBuffer"), top.title);
			outStats->insert(QStringLiteral("topBufferMax"), top.maxInventory);
		}
	}
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
	const bool includeTraces = config.value(QStringLiteral("includeTraces")).toBool(false);

	const QJsonObject flow = canvas->toJson();
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(flow, cfg);
	if (!built.ok)
	{
		if (outError)
			*outError = built.error;
		return false;
	}

	QVector<PolicyCompareRow> rows;
	QVector<SimStatistics> perPolicy;
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
		if (includeTraces)
			perPolicy.append(st);
	}

	m_plugin->applyAiCompareRows(rows);
	if (!perPolicy.isEmpty() && m_plugin)
	{
		// 对比甘特由 UI setCompareResult 承接；此处仅汇总
	}

	if (outRows)
	{
		QJsonArray arr;
		for (int i = 0; i < rows.size(); ++i)
		{
			const PolicyCompareRow& row = rows[i];
			QJsonObject o;
			o.insert(QStringLiteral("policy"), row.policy);
			o.insert(QStringLiteral("makespan"), row.makespan);
			o.insert(QStringLiteral("completed"), row.completed);
			o.insert(QStringLiteral("throughput"), row.throughput);
			o.insert(QStringLiteral("bottleneck"), row.bottleneck);
			if (includeTraces && i < perPolicy.size())
				o.insert(QStringLiteral("operationTrace"), perPolicy[i].trace.toJsonArray());
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
