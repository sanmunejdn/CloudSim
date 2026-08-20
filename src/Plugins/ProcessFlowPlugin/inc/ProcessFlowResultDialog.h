#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWRESULTDIALOG_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWRESULTDIALOG_H

/// @file ProcessFlowResultDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 甘特 / Trace / 策略对比（含可选多策略甘特）

#include "sim/SimStatistics.h"

#include <QDialog>
#include <QVector>

class ProcessFlowGanttWidget;
class QComboBox;
class QTableWidget;

class ProcessFlowResultDialog final : public QDialog
{
	Q_OBJECT

public:
	enum class Mode
	{
		Gantt,
		Trace,
		Compare
	};

	explicit ProcessFlowResultDialog(Mode mode, QWidget* parent = nullptr);

	Mode mode() const { return m_mode; }
	void applyLanguage(bool useChinese);
	void setStatistics(const SimStatistics& stats);
	void setCompareRows(const QVector<PolicyCompareRow>& rows);
	void setCompareStats(const QVector<SimStatistics>& stats);
	void clear();

private:
	void rebuildTrace(const SimStatistics& stats);
	void updateWindowTitle();
	void showCompareGantt(int index);

	Mode m_mode = Mode::Gantt;
	bool m_zh = true;
	ProcessFlowGanttWidget* m_gantt = nullptr;
	QTableWidget* m_table = nullptr;
	QComboBox* m_policyCombo = nullptr;
	ProcessFlowGanttWidget* m_compareGantt = nullptr;
	QVector<SimStatistics> m_compareStats;
};

#endif
