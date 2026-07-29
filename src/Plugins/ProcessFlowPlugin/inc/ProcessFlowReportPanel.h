#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWREPORTPANEL_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWREPORTPANEL_H

/// @file ProcessFlowReportPanel.h
/// @brief 仿真控制、统计摘要；甘特/Trace/对比走对话框

#include "ProcessFlowResultDialog.h"
#include "sim/SimRunConfig.h"
#include "sim/SimStatistics.h"

#include <QPointer>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QTableWidget;
class QTimer;

class ProcessFlowReportPanel final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowReportPanel(QWidget* parent = nullptr);

	void applyLanguage(bool useChinese);
	void setStatistics(const SimStatistics& stats);
	void setCompareRows(const QVector<PolicyCompareRow>& rows);
	void setCompareResult(const QVector<PolicyCompareRow>& rows, const QVector<SimStatistics>& perPolicy);
	void clearStatistics();
	void setRunning(bool running);
	void openGanttDialog();

	double horizonSec() const;
	double warmupSec() const;
	QString policyName() const;
	QString arrivalMode() const;
	bool openGanttAfterRun() const;
	bool includeCompareTraces() const;
	bool shiftEnabled() const;
	ShiftCalendar shiftCalendar() const;
	QStringList comparePolicies() const;
	const SimStatistics& lastStats() const { return m_stats; }
	void applyConfigTo(SimRunConfig* cfg) const;

signals:
	void runClicked();
	void compareClicked();
	void optimizeClicked();
	void stopClicked();
	void exportJsonClicked();
	void exportCsvClicked();
	void playbackTimeChanged(double t);
	void playbackLoadRequested();

private:
	void rebuildSummary(const SimStatistics& stats);
	void updateEntryEnabled();
	void openResultDialog(ProcessFlowResultDialog::Mode mode);
	void refreshOpenDialogs();

	QLabel* m_title = nullptr;
	QLabel* m_summary = nullptr;
	QDoubleSpinBox* m_horizon = nullptr;
	QDoubleSpinBox* m_warmup = nullptr;
	QDoubleSpinBox* m_interarrival = nullptr;
	QComboBox* m_policy = nullptr;
	QComboBox* m_arrival = nullptr;
	QCheckBox* m_shiftEnable = nullptr;
	QDoubleSpinBox* m_shiftStart = nullptr;
	QDoubleSpinBox* m_shiftEnd = nullptr;
	QCheckBox* m_openGantt = nullptr;
	QCheckBox* m_compareGantt = nullptr;
	QPushButton* m_runBtn = nullptr;
	QPushButton* m_optimizeBtn = nullptr;
	QPushButton* m_compareBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_exportJsonBtn = nullptr;
	QPushButton* m_exportCsvBtn = nullptr;
	QPushButton* m_ganttBtn = nullptr;
	QPushButton* m_traceBtn = nullptr;
	QPushButton* m_compareViewBtn = nullptr;
	QTableWidget* m_machineTable = nullptr;
	QTableWidget* m_bufferTable = nullptr;
	QSlider* m_playSlider = nullptr;
	QPushButton* m_playBtn = nullptr;
	QTimer* m_playTimer = nullptr;
	SimStatistics m_stats;
	QVector<PolicyCompareRow> m_compareRows;
	QVector<SimStatistics> m_compareStats;
	QPointer<ProcessFlowResultDialog> m_ganttDialog;
	QPointer<ProcessFlowResultDialog> m_traceDialog;
	QPointer<ProcessFlowResultDialog> m_compareDialog;
	bool m_zh = true;
	bool m_playing = false;
};

#endif
