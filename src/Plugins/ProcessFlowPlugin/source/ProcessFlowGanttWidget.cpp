/// @file ProcessFlowGanttWidget.cpp
/// @brief 甘特绘制

#include "ProcessFlowGanttWidget.h"

#include <QEvent>
#include <QHelpEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QToolTip>
#include <QWheelEvent>
#include <QtGlobal>
#include <algorithm>
#include <cmath>

ProcessFlowGanttWidget::ProcessFlowGanttWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setMinimumHeight(120);
	clear();
}

void ProcessFlowGanttWidget::clear()
{
	m_stats = SimStatistics();
	m_machineOrder.clear();
	m_machineTitles.clear();
	m_timeMax = 1.0;
	recomputeSize();
	update();
}

void ProcessFlowGanttWidget::setStatistics(const SimStatistics& stats)
{
	m_stats = stats;
	m_machineOrder.clear();
	m_machineTitles.clear();
	for (const MachineStat& m : stats.machines)
	{
		m_machineOrder.append(m.nodeId);
		m_machineTitles.insert(m.nodeId, m.title.isEmpty() ? QString::number(m.nodeId) : m.title);
	}
	for (const OperationTraceItem& it : stats.trace.items)
	{
		if (!m_machineTitles.contains(it.machineNodeId))
		{
			m_machineOrder.append(it.machineNodeId);
			m_machineTitles.insert(it.machineNodeId, QString::number(it.machineNodeId));
		}
	}
	std::sort(m_machineOrder.begin(), m_machineOrder.end());
	m_machineOrder.erase(std::unique(m_machineOrder.begin(), m_machineOrder.end()), m_machineOrder.end());

	m_timeMax = std::max(1.0, std::max(stats.makespan, stats.horizonSec));
	for (const OperationTraceItem& it : stats.trace.items)
	{
		m_timeMax = std::max(m_timeMax, it.end);
	}
	recomputeSize();
	update();
}

void ProcessFlowGanttWidget::recomputeSize()
{
	const int rows = std::max(1, m_machineOrder.size());
	const int w = m_labelW + static_cast<int>(std::ceil(m_timeMax * m_pxPerSec)) + 24;
	const int h = m_topPad + rows * m_rowH + 16;
	setMinimumSize(w, h);
	resize(w, h);
}

QRectF ProcessFlowGanttWidget::barRect(int row, double start, double end) const
{
	const double x0 = m_labelW + start * m_pxPerSec;
	const double x1 = m_labelW + end * m_pxPerSec;
	const double y = m_topPad + row * m_rowH + 4;
	return QRectF(x0, y, std::max(2.0, x1 - x0), m_rowH - 8);
}

int ProcessFlowGanttWidget::hitBar(const QPoint& pos, OperationTraceItem* out) const
{
	for (int r = 0; r < m_machineOrder.size(); ++r)
	{
		const int mid = m_machineOrder[r];
		for (const OperationTraceItem& it : m_stats.trace.items)
		{
			if (it.machineNodeId != mid)
			{
				continue;
			}
			if (barRect(r, it.start, it.end).contains(pos))
			{
				if (out)
				{
					*out = it;
				}
				return it.jobId;
			}
		}
	}
	return -1;
}

QSize ProcessFlowGanttWidget::sizeHint() const
{
	return minimumSize();
}

QSize ProcessFlowGanttWidget::minimumSizeHint() const
{
	return minimumSize();
}

void ProcessFlowGanttWidget::wheelEvent(QWheelEvent* event)
{
	const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
	m_pxPerSec = std::clamp(m_pxPerSec * factor, 0.2, 40.0);
	recomputeSize();
	update();
	event->accept();
}

void ProcessFlowGanttWidget::mouseMoveEvent(QMouseEvent* event)
{
	update();
	QWidget::mouseMoveEvent(event);
}

bool ProcessFlowGanttWidget::event(QEvent* event)
{
	if (event->type() == QEvent::ToolTip)
	{
		auto* help = static_cast<QHelpEvent*>(event);
		OperationTraceItem it;
		if (hitBar(help->pos(), &it) >= 0)
		{
			QToolTip::showText(help->globalPos(),
							   QStringLiteral("Job %1  Op %2\n[%3, %4]")
								   .arg(it.jobId)
								   .arg(it.opSeq)
								   .arg(it.start, 0, 'f', 2)
								   .arg(it.end, 0, 'f', 2),
							   this);
			return true;
		}
		QToolTip::hideText();
	}
	return QWidget::event(event);
}

void ProcessFlowGanttWidget::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.fillRect(rect(), QColor(QStringLiteral("#F8FAFC")));
	p.setPen(QColor(QStringLiteral("#94A3B8")));
	p.drawText(8, 16, QStringLiteral("0 … %1s  (滚轮缩放)").arg(m_timeMax, 0, 'f', 0));

	const int chartW = width() - m_labelW - 8;
	for (int i = 0; i <= 4; ++i)
	{
		const double t = m_timeMax * i / 4.0;
		const int x = m_labelW + static_cast<int>(t * m_pxPerSec);
		p.setPen(QPen(QColor(QStringLiteral("#E2E8F0")), 1, Qt::DashLine));
		p.drawLine(x, m_topPad - 4, x, height() - 8);
		p.setPen(QColor(QStringLiteral("#64748B")));
		p.drawText(x + 2, m_topPad - 8, QString::number(t, 'f', 0));
		Q_UNUSED(chartW);
	}

	for (int r = 0; r < m_machineOrder.size(); ++r)
	{
		const int mid = m_machineOrder[r];
		const int y = m_topPad + r * m_rowH;
		p.fillRect(0, y, m_labelW - 4, m_rowH, QColor(QStringLiteral("#EEF2FF")));
		p.setPen(QColor(QStringLiteral("#1E293B")));
		p.drawText(QRect(4, y, m_labelW - 8, m_rowH), Qt::AlignVCenter | Qt::AlignLeft,
				   m_machineTitles.value(mid));
		p.setPen(QColor(QStringLiteral("#E2E8F0")));
		p.drawLine(0, y + m_rowH, width(), y + m_rowH);

		for (const OperationTraceItem& it : m_stats.trace.items)
		{
			if (it.machineNodeId != mid)
			{
				continue;
			}
			const int hue = (it.jobId * 47) % 360;
			QColor fill = QColor::fromHsv(hue, 140, 220);
			fill.setAlpha(220);
			const QRectF rct = barRect(r, it.start, it.end);
			p.setPen(Qt::NoPen);
			p.setBrush(fill);
			p.drawRoundedRect(rct, 3, 3);
			if (rct.width() > 28)
			{
				p.setPen(QColor(QStringLiteral("#0F172A")));
				p.drawText(rct, Qt::AlignCenter, QStringLiteral("J%1").arg(it.jobId));
			}
		}
	}
	if (m_machineOrder.isEmpty())
	{
		p.setPen(QColor(QStringLiteral("#94A3B8")));
		p.drawText(rect(), Qt::AlignCenter, QStringLiteral("暂无甘特数据"));
	}
}
