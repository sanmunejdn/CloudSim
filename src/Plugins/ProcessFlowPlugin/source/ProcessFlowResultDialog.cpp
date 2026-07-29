/// @file ProcessFlowResultDialog.cpp
/// @brief 结果大图对话框

#include "ProcessFlowResultDialog.h"

#include "ProcessFlowGanttWidget.h"

#include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QScrollArea>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>

ProcessFlowResultDialog::ProcessFlowResultDialog(Mode mode, QWidget* parent) : QDialog(parent), m_mode(mode)
{
	setAttribute(Qt::WA_DeleteOnClose, false);
	setWindowFlags(windowFlags() | Qt::Window);
	resize(900, 560);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(8, 8, 8, 8);

	if (m_mode == Mode::Gantt)
	{
		m_gantt = new ProcessFlowGanttWidget(this);
		auto* scroll = new QScrollArea(this);
		scroll->setWidget(m_gantt);
		scroll->setWidgetResizable(false);
		scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
		scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
		layout->addWidget(scroll);
	}
	else if (m_mode == Mode::Trace)
	{
		m_table = new QTableWidget(0, 5, this);
		m_table->setHorizontalHeaderLabels(
			{QStringLiteral("Job"), QStringLiteral("Op"), QStringLiteral("机器"), QStringLiteral("开始"),
			 QStringLiteral("结束")});
		m_table->horizontalHeader()->setStretchLastSection(true);
		layout->addWidget(m_table);
	}
	else
	{
		m_table = new QTableWidget(0, 5, this);
		m_table->setHorizontalHeaderLabels(
			{QStringLiteral("策略"), QStringLiteral("Makespan"), QStringLiteral("完成"), QStringLiteral("吞吐"),
			 QStringLiteral("瓶颈")});
		m_table->horizontalHeader()->setStretchLastSection(true);
		layout->addWidget(m_table);

		layout->addWidget(new QLabel(QStringLiteral("甘特策略"), this));
		m_policyCombo = new QComboBox(this);
		layout->addWidget(m_policyCombo);
		m_compareGantt = new ProcessFlowGanttWidget(this);
		auto* scroll = new QScrollArea(this);
		scroll->setWidget(m_compareGantt);
		scroll->setWidgetResizable(false);
		layout->addWidget(scroll, 1);
		connect(m_policyCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				[this](int i) { showCompareGantt(i); });
	}
	updateWindowTitle();
}

void ProcessFlowResultDialog::applyLanguage(bool useChinese)
{
	m_zh = useChinese;
	updateWindowTitle();
	if (m_mode == Mode::Trace && m_table)
	{
		m_table->setHorizontalHeaderLabels(
			{QStringLiteral("Job"), QStringLiteral("Op"), useChinese ? QStringLiteral("机器") : QStringLiteral("Machine"),
			 useChinese ? QStringLiteral("开始") : QStringLiteral("Start"),
			 useChinese ? QStringLiteral("结束") : QStringLiteral("End")});
	}
	else if (m_mode == Mode::Compare && m_table)
	{
		m_table->setHorizontalHeaderLabels(
			{useChinese ? QStringLiteral("策略") : QStringLiteral("Policy"), QStringLiteral("Makespan"),
			 useChinese ? QStringLiteral("完成") : QStringLiteral("Done"),
			 useChinese ? QStringLiteral("吞吐") : QStringLiteral("Throughput"),
			 useChinese ? QStringLiteral("瓶颈") : QStringLiteral("Bottleneck")});
	}
}

void ProcessFlowResultDialog::updateWindowTitle()
{
	if (m_mode == Mode::Gantt)
		setWindowTitle(m_zh ? QStringLiteral("甘特图") : QStringLiteral("Gantt Chart"));
	else if (m_mode == Mode::Trace)
		setWindowTitle(m_zh ? QStringLiteral("操作 Trace") : QStringLiteral("Operation Trace"));
	else
		setWindowTitle(m_zh ? QStringLiteral("策略对比") : QStringLiteral("Policy Compare"));
}

void ProcessFlowResultDialog::setStatistics(const SimStatistics& stats)
{
	if (m_mode == Mode::Gantt && m_gantt)
		m_gantt->setStatistics(stats);
	else if (m_mode == Mode::Trace)
		rebuildTrace(stats);
}

void ProcessFlowResultDialog::setCompareRows(const QVector<PolicyCompareRow>& rows)
{
	if (m_mode != Mode::Compare || !m_table)
		return;
	m_table->setRowCount(rows.size());
	for (int r = 0; r < rows.size(); ++r)
	{
		const PolicyCompareRow& row = rows[r];
		m_table->setItem(r, 0, new QTableWidgetItem(row.policy));
		m_table->setItem(r, 1, new QTableWidgetItem(QString::number(row.makespan, 'f', 1)));
		m_table->setItem(r, 2, new QTableWidgetItem(QString::number(row.completed)));
		m_table->setItem(r, 3, new QTableWidgetItem(QString::number(row.throughput, 'f', 2)));
		m_table->setItem(r, 4, new QTableWidgetItem(row.bottleneck));
	}
}

void ProcessFlowResultDialog::setCompareStats(const QVector<SimStatistics>& stats)
{
	m_compareStats = stats;
	if (!m_policyCombo)
		return;
	m_policyCombo->blockSignals(true);
	m_policyCombo->clear();
	for (int i = 0; i < m_compareStats.size(); ++i)
	{
		QString label = QStringLiteral("Policy-%1").arg(i + 1);
		if (m_table && i < m_table->rowCount() && m_table->item(i, 0))
			label = m_table->item(i, 0)->text();
		m_policyCombo->addItem(label);
	}
	m_policyCombo->blockSignals(false);
	if (!m_compareStats.isEmpty())
		showCompareGantt(0);
}

void ProcessFlowResultDialog::showCompareGantt(int index)
{
	if (!m_compareGantt || index < 0 || index >= m_compareStats.size())
		return;
	m_compareGantt->setStatistics(m_compareStats.at(index));
}

void ProcessFlowResultDialog::rebuildTrace(const SimStatistics& stats)
{
	if (!m_table)
		return;
	const int n = stats.trace.items.size();
	const int show = std::min(n, 2000);
	m_table->setRowCount(show);
	for (int r = 0; r < show; ++r)
	{
		const OperationTraceItem& it = stats.trace.items[r];
		m_table->setItem(r, 0, new QTableWidgetItem(QString::number(it.jobId)));
		m_table->setItem(r, 1, new QTableWidgetItem(QString::number(it.opSeq)));
		m_table->setItem(r, 2, new QTableWidgetItem(QString::number(it.machineNodeId)));
		m_table->setItem(r, 3, new QTableWidgetItem(QString::number(it.start, 'f', 2)));
		m_table->setItem(r, 4, new QTableWidgetItem(QString::number(it.end, 'f', 2)));
	}
}

void ProcessFlowResultDialog::clear()
{
	if (m_gantt)
		m_gantt->clear();
	if (m_compareGantt)
		m_compareGantt->clear();
	if (m_table)
		m_table->setRowCount(0);
	m_compareStats.clear();
	if (m_policyCombo)
		m_policyCombo->clear();
}
