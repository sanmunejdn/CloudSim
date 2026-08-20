#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWGANTTWIDGET_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWGANTTWIDGET_H

/// @file ProcessFlowGanttWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief OperationTrace 机器×时间甘特

#include "sim/SimStatistics.h"

#include <QHash>
#include <QWidget>

class QScrollArea;

class ProcessFlowGanttWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowGanttWidget(QWidget* parent = nullptr);

	void setStatistics(const SimStatistics& stats);
	void clear();

protected:
	void paintEvent(QPaintEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	bool event(QEvent* event) override;
	QSize sizeHint() const override;
	QSize minimumSizeHint() const override;

private:
	QRectF barRect(int row, double start, double end) const;
	int hitBar(const QPoint& pos, OperationTraceItem* out) const;
	void recomputeSize();

	SimStatistics m_stats;
	QVector<int> m_machineOrder;
	QHash<int, QString> m_machineTitles;
	double m_timeMax = 1.0;
	double m_pxPerSec = 2.0;
	int m_labelW = 88;
	int m_rowH = 28;
	int m_topPad = 24;
};

#endif
