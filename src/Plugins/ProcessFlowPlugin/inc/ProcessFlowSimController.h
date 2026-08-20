#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWSIMCONTROLLER_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWSIMCONTROLLER_H

/// @file ProcessFlowSimController.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 仿真启停、对比、优化启发式与后台运行

#include "sim/SimRunConfig.h"
#include "sim/SimStatistics.h"

#include <QObject>
#include <QStringList>
#include <QVector>
#include <atomic>
#include <memory>

class IPluginHostContext;
class ProcessFlowCanvasWidget;

class ProcessFlowSimController : public QObject
{
	Q_OBJECT

public:
	explicit ProcessFlowSimController(QObject* parent = nullptr);

	void setHost(IPluginHostContext* host);
	bool isRunning() const { return m_running; }
	const SimStatistics& lastResult() const { return m_lastResult; }
	const QVector<SimStatistics>& lastCompareStats() const { return m_lastCompareStats; }
	SimRunConfig& config() { return m_config; }
	const SimRunConfig& config() const { return m_config; }

public slots:
	void start(ProcessFlowCanvasWidget* canvas);
	void compare(ProcessFlowCanvasWidget* canvas, const QStringList& policies);
	void optimizeThenStart(ProcessFlowCanvasWidget* canvas);
	void stop();
	void clearResult();

signals:
	void started();
	void finished(bool ok, const QString& message);
	void resultReady(const SimStatistics& stats);
	void compareReady(const QVector<PolicyCompareRow>& rows, const QVector<SimStatistics>& perPolicy);

private:
	void runInternal(ProcessFlowCanvasWidget* canvas, const QStringList& policies, bool compareMode);

	IPluginHostContext* m_host = nullptr;
	SimRunConfig m_config;
	SimStatistics m_lastResult;
	QVector<SimStatistics> m_lastCompareStats;
	std::shared_ptr<std::atomic_bool> m_cancel;
	bool m_running = false;
};

#endif
