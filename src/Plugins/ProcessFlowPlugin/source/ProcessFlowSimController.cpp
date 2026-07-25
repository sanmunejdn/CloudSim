/// @file ProcessFlowSimController.cpp
/// @brief 从图快照后台跑 DES / 多策略对比

#include "ProcessFlowSimController.h"

#include "IPluginHostContext.h"
#include "ProcessFlowCanvasWidget.h"
#include "sim/DesEngine.h"
#include "sim/DispatchPolicies.h"
#include "sim/IStationExecutor.h"
#include "sim/SimModelBuilder.h"

#include <QJsonObject>
#include <QVector>

#include <exception>
#include <memory>

ProcessFlowSimController::ProcessFlowSimController(QObject* parent) : QObject(parent) {}

void ProcessFlowSimController::setHost(IPluginHostContext* host)
{
	m_host = host;
}

void ProcessFlowSimController::clearResult()
{
	m_lastResult = SimStatistics();
}

void ProcessFlowSimController::stop()
{
	if (m_cancel)
		m_cancel->store(true);
}

void ProcessFlowSimController::start(ProcessFlowCanvasWidget* canvas)
{
	runInternal(canvas, {m_config.policy}, false);
}

void ProcessFlowSimController::compare(ProcessFlowCanvasWidget* canvas, const QStringList& policies)
{
	runInternal(canvas, policies.isEmpty() ? allDispatchPolicyNames() : policies, true);
}

void ProcessFlowSimController::runInternal(ProcessFlowCanvasWidget* canvas, const QStringList& policies, bool compareMode)
{
	if (!m_host || !canvas || m_running)
		return;
	const QJsonObject flow = canvas->toJson();
	const SimBuildResult built = SimModelBuilder::fromProcessFlowJson(flow, m_config);
	if (!built.ok)
	{
		emit finished(false, built.error);
		return;
	}

	m_running = true;
	m_cancel = std::make_shared<std::atomic_bool>(false);
	emit started();

	const SimRunConfig cfg = m_config;
	const PlantGraph plant = built.plant;
	const JobSet jobSet = built.jobSet;
	const double interarrival = built.interarrivalSec;
	auto cancel = m_cancel;
	auto resultHolder = std::make_shared<SimStatistics>();
	auto compareHolder = std::make_shared<QVector<PolicyCompareRow>>();
	auto errorHolder = std::make_shared<QString>();
	const QStringList pols = policies;

	m_host->enqueueJob(
		QStringLiteral("ProcessFlow DES"),
		[plant, jobSet, interarrival, cfg, cancel, pols, compareMode, resultHolder, compareHolder,
		 errorHolder](const PluginJobProgressFn& progress)
		{
			try
			{
				int step = 0;
				for (const QString& policyName : pols)
				{
					if (cancel && cancel->load())
						break;
					if (progress)
						progress(5 + (90 * step) / std::max(1, pols.size()), policyName);
					DesEngine engine;
					engine.setDispatchPolicy(createDispatchPolicy(policyName));
					engine.setStationExecutor(std::make_unique<NullStationExecutor>());
					SimRunConfig c = cfg;
					c.policy = policyName;
					SimStatistics st = engine.run(plant, jobSet, interarrival, c, cancel.get());
					PolicyCompareRow row;
					row.policy = policyName.toUpper();
					row.makespan = st.makespan;
					row.completed = st.completedJobs;
					row.throughput = st.throughputPerHour;
					row.bottleneck = st.bottleneckTitle.isEmpty() ? QString::number(st.bottleneckNodeId)
																 : st.bottleneckTitle;
					compareHolder->append(row);
					*resultHolder = st; // 保留最后一次详细结果
					++step;
				}
				if (progress)
					progress(100, QStringLiteral("done"));
				Q_UNUSED(compareMode);
			}
			catch (const std::exception& ex)
			{
				*errorHolder = QString::fromUtf8(ex.what());
			}
			catch (...)
			{
				*errorHolder = QStringLiteral("DES unknown error");
			}
		},
		[this, resultHolder, compareHolder, errorHolder, cancel, compareMode](bool threw, const QString& throwMessage)
		{
			m_running = false;
			const bool cancelled = cancel && cancel->load();
			if (threw)
			{
				emit finished(false, throwMessage.isEmpty() ? *errorHolder : throwMessage);
				return;
			}
			if (!errorHolder->isEmpty())
			{
				emit finished(false, *errorHolder);
				return;
			}
			if (cancelled)
			{
				emit finished(false, QStringLiteral("cancelled"));
				return;
			}
			m_lastResult = *resultHolder;
			emit resultReady(m_lastResult);
			if (compareMode)
				emit compareReady(*compareHolder);
			emit finished(true, QStringLiteral("ok"));
		});
}
