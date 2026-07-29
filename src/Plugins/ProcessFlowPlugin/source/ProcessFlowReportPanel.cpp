/// @file ProcessFlowReportPanel.cpp
/// @brief 报表摘要 + 结果对话框入口

#include "ProcessFlowReportPanel.h"

#include "ProcessFlowResultDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

ProcessFlowReportPanel::ProcessFlowReportPanel(QWidget* parent) : QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 8, 0, 0);
	layout->setSpacing(6);

	m_title = new QLabel(QStringLiteral("仿真报表"), this);
	m_title->setObjectName(QStringLiteral("ProcessFlowSectionTitle"));

	auto* form = new QFormLayout();
	m_horizon = new QDoubleSpinBox(this);
	m_horizon->setRange(1.0, 1e7);
	m_horizon->setValue(3600.0);
	m_horizon->setSuffix(QStringLiteral(" s"));
	m_warmup = new QDoubleSpinBox(this);
	m_warmup->setRange(0.0, 1e7);
	m_warmup->setSuffix(QStringLiteral(" s"));
	m_interarrival = new QDoubleSpinBox(this);
	m_interarrival->setRange(0.1, 1e6);
	m_interarrival->setValue(30.0);
	m_interarrival->setSuffix(QStringLiteral(" s"));
	m_policy = new QComboBox(this);
	m_policy->addItem(QStringLiteral("FIFO"), QStringLiteral("fifo"));
	m_policy->addItem(QStringLiteral("SPT"), QStringLiteral("spt"));
	m_policy->addItem(QStringLiteral("LPT"), QStringLiteral("lpt"));
	m_policy->addItem(QStringLiteral("EDD"), QStringLiteral("edd"));
	m_policy->addItem(QStringLiteral("CR"), QStringLiteral("cr"));
	m_arrival = new QComboBox(this);
	m_arrival->addItem(QStringLiteral("固定间隔"), QStringLiteral("fixed"));
	m_arrival->addItem(QStringLiteral("指数分布"), QStringLiteral("exponential"));
	m_shiftEnable = new QCheckBox(QStringLiteral("启用班次"), this);
	m_shiftStart = new QDoubleSpinBox(this);
	m_shiftStart->setRange(0.0, 86400.0);
	m_shiftStart->setValue(28800.0);
	m_shiftStart->setSuffix(QStringLiteral(" s"));
	m_shiftEnd = new QDoubleSpinBox(this);
	m_shiftEnd->setRange(0.0, 86400.0);
	m_shiftEnd->setValue(57600.0);
	m_shiftEnd->setSuffix(QStringLiteral(" s"));
	m_openGantt = new QCheckBox(QStringLiteral("完成后打开甘特"), this);
	m_compareGantt = new QCheckBox(QStringLiteral("对比含甘特"), this);
	m_compareGantt->setChecked(true);
	form->addRow(QStringLiteral("时长"), m_horizon);
	form->addRow(QStringLiteral("Warmup"), m_warmup);
	form->addRow(QStringLiteral("到达间隔"), m_interarrival);
	form->addRow(QStringLiteral("到达模式"), m_arrival);
	form->addRow(QStringLiteral("策略"), m_policy);
	form->addRow(QString(), m_shiftEnable);
	form->addRow(QStringLiteral("班次起"), m_shiftStart);
	form->addRow(QStringLiteral("班次止"), m_shiftEnd);
	form->addRow(QString(), m_openGantt);
	form->addRow(QString(), m_compareGantt);

	auto* btnRow = new QHBoxLayout();
	m_runBtn = new QPushButton(QStringLiteral("运行"), this);
	m_runBtn->setObjectName(QStringLiteral("ProcessFlowPrimaryButton"));
	m_optimizeBtn = new QPushButton(QStringLiteral("优化后仿真"), this);
	m_compareBtn = new QPushButton(QStringLiteral("对比全部策略"), this);
	m_stopBtn = new QPushButton(QStringLiteral("停止"), this);
	m_stopBtn->setEnabled(false);
	m_exportJsonBtn = new QPushButton(QStringLiteral("导出JSON"), this);
	m_exportCsvBtn = new QPushButton(QStringLiteral("导出CSV"), this);
	btnRow->addWidget(m_runBtn);
	btnRow->addWidget(m_optimizeBtn);
	btnRow->addWidget(m_compareBtn);
	btnRow->addWidget(m_stopBtn);

	auto* exportRow = new QHBoxLayout();
	exportRow->addWidget(m_exportJsonBtn);
	exportRow->addWidget(m_exportCsvBtn);

	m_summary = new QLabel(QStringLiteral("尚未运行"), this);
	m_summary->setWordWrap(true);

	m_machineTable = new QTableWidget(0, 5, this);
	m_machineTable->setHorizontalHeaderLabels(
		{QStringLiteral("节点"), QStringLiteral("利用率"), QStringLiteral("忙时"), QStringLiteral("阻塞"),
		 QStringLiteral("均队长")});
	m_machineTable->horizontalHeader()->setStretchLastSection(true);
	m_machineTable->setMaximumHeight(120);

	m_bufferTable = new QTableWidget(0, 4, this);
	m_bufferTable->setHorizontalHeaderLabels(
		{QStringLiteral("缓冲段"), QStringLiteral("均库存"), QStringLiteral("峰库存"), QStringLiteral("满次")});
	m_bufferTable->horizontalHeader()->setStretchLastSection(true);
	m_bufferTable->setMaximumHeight(100);

	auto* resultRow = new QHBoxLayout();
	m_ganttBtn = new QPushButton(QStringLiteral("甘特图…"), this);
	m_traceBtn = new QPushButton(QStringLiteral("操作 Trace…"), this);
	m_compareViewBtn = new QPushButton(QStringLiteral("策略对比…"), this);
	resultRow->addWidget(m_ganttBtn);
	resultRow->addWidget(m_traceBtn);
	resultRow->addWidget(m_compareViewBtn);

	auto* playRow = new QHBoxLayout();
	m_playBtn = new QPushButton(QStringLiteral("回放"), this);
	m_playSlider = new QSlider(Qt::Horizontal, this);
	m_playSlider->setRange(0, 1000);
	playRow->addWidget(m_playBtn);
	playRow->addWidget(m_playSlider, 1);
	m_playTimer = new QTimer(this);
	m_playTimer->setInterval(50);

	layout->addWidget(m_title);
	layout->addLayout(form);
	layout->addLayout(btnRow);
	layout->addLayout(exportRow);
	layout->addWidget(m_summary);
	layout->addWidget(new QLabel(QStringLiteral("机器统计"), this));
	layout->addWidget(m_machineTable);
	layout->addWidget(new QLabel(QStringLiteral("缓冲统计"), this));
	layout->addWidget(m_bufferTable);
	layout->addWidget(new QLabel(QStringLiteral("结果查看"), this));
	layout->addLayout(resultRow);
	layout->addLayout(playRow);
	layout->addStretch(1);

	updateEntryEnabled();

	connect(m_runBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::runClicked);
	connect(m_optimizeBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::optimizeClicked);
	connect(m_compareBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::compareClicked);
	connect(m_stopBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::stopClicked);
	connect(m_exportJsonBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::exportJsonClicked);
	connect(m_exportCsvBtn, &QPushButton::clicked, this, &ProcessFlowReportPanel::exportCsvClicked);
	connect(m_ganttBtn, &QPushButton::clicked, this, [this]() { openResultDialog(ProcessFlowResultDialog::Mode::Gantt); });
	connect(m_traceBtn, &QPushButton::clicked, this, [this]() { openResultDialog(ProcessFlowResultDialog::Mode::Trace); });
	connect(m_compareViewBtn, &QPushButton::clicked, this,
			[this]() { openResultDialog(ProcessFlowResultDialog::Mode::Compare); });
	connect(m_playBtn, &QPushButton::clicked, this,
			[this]()
			{
				emit playbackLoadRequested();
				m_playing = !m_playing;
				m_playBtn->setText(m_playing ? (m_zh ? QStringLiteral("暂停") : QStringLiteral("Pause"))
											 : (m_zh ? QStringLiteral("回放") : QStringLiteral("Replay")));
				if (m_playing)
					m_playTimer->start();
				else
					m_playTimer->stop();
			});
	connect(m_playSlider, &QSlider::valueChanged, this,
			[this](int v)
			{
				const double tmax = std::max(1.0, std::max(m_stats.makespan, m_stats.horizonSec));
				emit playbackTimeChanged(tmax * (v / 1000.0));
			});
	connect(m_playTimer, &QTimer::timeout, this,
			[this]()
			{
				int v = m_playSlider->value() + 4;
				if (v >= 1000)
				{
					v = 1000;
					m_playing = false;
					m_playTimer->stop();
					m_playBtn->setText(m_zh ? QStringLiteral("回放") : QStringLiteral("Replay"));
				}
				m_playSlider->setValue(v);
			});
}

void ProcessFlowReportPanel::applyLanguage(bool useChinese)
{
	m_zh = useChinese;
	m_title->setText(useChinese ? QStringLiteral("仿真报表") : QStringLiteral("Simulation Report"));
	m_runBtn->setText(useChinese ? QStringLiteral("运行") : QStringLiteral("Run"));
	m_optimizeBtn->setText(useChinese ? QStringLiteral("优化后仿真") : QStringLiteral("Optimize+Run"));
	m_compareBtn->setText(useChinese ? QStringLiteral("对比全部策略") : QStringLiteral("Compare Policies"));
	m_stopBtn->setText(useChinese ? QStringLiteral("停止") : QStringLiteral("Stop"));
	m_exportJsonBtn->setText(useChinese ? QStringLiteral("导出JSON") : QStringLiteral("Export JSON"));
	m_exportCsvBtn->setText(useChinese ? QStringLiteral("导出CSV") : QStringLiteral("Export CSV"));
	m_ganttBtn->setText(useChinese ? QStringLiteral("甘特图…") : QStringLiteral("Gantt…"));
	m_traceBtn->setText(useChinese ? QStringLiteral("操作 Trace…") : QStringLiteral("Trace…"));
	m_compareViewBtn->setText(useChinese ? QStringLiteral("策略对比…") : QStringLiteral("Compare…"));
	m_openGantt->setText(useChinese ? QStringLiteral("完成后打开甘特") : QStringLiteral("Open Gantt after run"));
	m_compareGantt->setText(useChinese ? QStringLiteral("对比含甘特") : QStringLiteral("Compare with Gantt"));
	m_shiftEnable->setText(useChinese ? QStringLiteral("启用班次") : QStringLiteral("Enable shift"));
	m_machineTable->setHorizontalHeaderLabels(
		useChinese ? QStringList{QStringLiteral("节点"), QStringLiteral("利用率"), QStringLiteral("忙时"),
								 QStringLiteral("阻塞"), QStringLiteral("均队长")}
				   : QStringList{QStringLiteral("Node"), QStringLiteral("Util"), QStringLiteral("Busy"),
								 QStringLiteral("Block"), QStringLiteral("Queue")});
	m_bufferTable->setHorizontalHeaderLabels(
		useChinese ? QStringList{QStringLiteral("缓冲段"), QStringLiteral("均库存"), QStringLiteral("峰库存"),
								 QStringLiteral("满次")}
				   : QStringList{QStringLiteral("Buffer"), QStringLiteral("Avg"), QStringLiteral("Max"),
								 QStringLiteral("Full")});
	if (m_ganttDialog)
		m_ganttDialog->applyLanguage(useChinese);
	if (m_traceDialog)
		m_traceDialog->applyLanguage(useChinese);
	if (m_compareDialog)
		m_compareDialog->applyLanguage(useChinese);
}

void ProcessFlowReportPanel::setRunning(bool running)
{
	m_runBtn->setEnabled(!running);
	m_optimizeBtn->setEnabled(!running);
	m_compareBtn->setEnabled(!running);
	m_stopBtn->setEnabled(running);
	m_horizon->setEnabled(!running);
	m_policy->setEnabled(!running);
}

void ProcessFlowReportPanel::clearStatistics()
{
	m_stats = SimStatistics();
	m_compareRows.clear();
	m_compareStats.clear();
	m_summary->setText(m_zh ? QStringLiteral("尚未运行") : QStringLiteral("No result"));
	m_machineTable->setRowCount(0);
	m_bufferTable->setRowCount(0);
	updateEntryEnabled();
	if (m_ganttDialog)
		m_ganttDialog->clear();
	if (m_traceDialog)
		m_traceDialog->clear();
	if (m_compareDialog)
		m_compareDialog->clear();
}

void ProcessFlowReportPanel::setStatistics(const SimStatistics& stats)
{
	m_stats = stats;
	rebuildSummary(stats);
	updateEntryEnabled();
	refreshOpenDialogs();
	if (openGanttAfterRun() && !stats.trace.items.isEmpty())
		openGanttDialog();
}

void ProcessFlowReportPanel::setCompareRows(const QVector<PolicyCompareRow>& rows)
{
	setCompareResult(rows, {});
}

void ProcessFlowReportPanel::setCompareResult(const QVector<PolicyCompareRow>& rows,
											 const QVector<SimStatistics>& perPolicy)
{
	m_compareRows = rows;
	m_compareStats = perPolicy;
	updateEntryEnabled();
	if (m_compareDialog)
	{
		m_compareDialog->setCompareRows(rows);
		m_compareDialog->setCompareStats(perPolicy);
	}
}

void ProcessFlowReportPanel::updateEntryEnabled()
{
	const bool hasTrace = !m_stats.trace.items.isEmpty();
	const bool hasCompare = !m_compareRows.isEmpty();
	if (m_ganttBtn)
		m_ganttBtn->setEnabled(hasTrace);
	if (m_traceBtn)
		m_traceBtn->setEnabled(hasTrace);
	if (m_compareViewBtn)
		m_compareViewBtn->setEnabled(hasCompare);
}

void ProcessFlowReportPanel::openGanttDialog()
{
	openResultDialog(ProcessFlowResultDialog::Mode::Gantt);
}

void ProcessFlowReportPanel::openResultDialog(ProcessFlowResultDialog::Mode mode)
{
	if (mode == ProcessFlowResultDialog::Mode::Compare)
	{
		if (m_compareRows.isEmpty())
		{
			QMessageBox::information(this, m_zh ? QStringLiteral("策略对比") : QStringLiteral("Compare"),
									 m_zh ? QStringLiteral("请先点击「对比全部策略」。")
										  : QStringLiteral("Run compare first."));
			return;
		}
		if (!m_compareDialog)
		{
			m_compareDialog = new ProcessFlowResultDialog(mode, this);
			m_compareDialog->applyLanguage(m_zh);
		}
		m_compareDialog->setCompareRows(m_compareRows);
		m_compareDialog->setCompareStats(m_compareStats);
		m_compareDialog->show();
		m_compareDialog->raise();
		m_compareDialog->activateWindow();
		return;
	}

	if (m_stats.trace.items.isEmpty())
	{
		QMessageBox::information(this, m_zh ? QStringLiteral("仿真结果") : QStringLiteral("Results"),
								 m_zh ? QStringLiteral("请先运行仿真。") : QStringLiteral("Run simulation first."));
		return;
	}

	QPointer<ProcessFlowResultDialog>& slot =
		(mode == ProcessFlowResultDialog::Mode::Gantt) ? m_ganttDialog : m_traceDialog;
	if (!slot)
	{
		slot = new ProcessFlowResultDialog(mode, this);
		slot->applyLanguage(m_zh);
	}
	slot->setStatistics(m_stats);
	slot->show();
	slot->raise();
	slot->activateWindow();
}

void ProcessFlowReportPanel::refreshOpenDialogs()
{
	if (m_ganttDialog)
		m_ganttDialog->setStatistics(m_stats);
	if (m_traceDialog)
		m_traceDialog->setStatistics(m_stats);
	if (m_compareDialog)
	{
		m_compareDialog->setCompareRows(m_compareRows);
		m_compareDialog->setCompareStats(m_compareStats);
	}
}

double ProcessFlowReportPanel::horizonSec() const
{
	return m_horizon ? m_horizon->value() : 3600.0;
}

double ProcessFlowReportPanel::warmupSec() const
{
	return m_warmup ? m_warmup->value() : 0.0;
}

QString ProcessFlowReportPanel::policyName() const
{
	return m_policy ? m_policy->currentData().toString() : QStringLiteral("fifo");
}

QString ProcessFlowReportPanel::arrivalMode() const
{
	return m_arrival ? m_arrival->currentData().toString() : QStringLiteral("fixed");
}

bool ProcessFlowReportPanel::openGanttAfterRun() const
{
	return m_openGantt && m_openGantt->isChecked();
}

bool ProcessFlowReportPanel::includeCompareTraces() const
{
	return m_compareGantt && m_compareGantt->isChecked();
}

bool ProcessFlowReportPanel::shiftEnabled() const
{
	return m_shiftEnable && m_shiftEnable->isChecked();
}

ShiftCalendar ProcessFlowReportPanel::shiftCalendar() const
{
	ShiftCalendar c;
	c.enabled = shiftEnabled();
	c.workStartSec = m_shiftStart ? m_shiftStart->value() : 28800.0;
	c.workEndSec = m_shiftEnd ? m_shiftEnd->value() : 57600.0;
	return c;
}

void ProcessFlowReportPanel::applyConfigTo(SimRunConfig* cfg) const
{
	if (!cfg)
		return;
	cfg->horizonSec = horizonSec();
	cfg->warmupSec = warmupSec();
	cfg->policy = policyName();
	cfg->arrivalMode = arrivalMode();
	cfg->defaultInterarrivalSec = m_interarrival ? m_interarrival->value() : 30.0;
	cfg->shift = shiftCalendar();
	cfg->openGanttAfterRun = openGanttAfterRun();
	cfg->includeCompareTraces = includeCompareTraces();
}

QStringList ProcessFlowReportPanel::comparePolicies() const
{
	return {QStringLiteral("fifo"), QStringLiteral("spt"), QStringLiteral("lpt"), QStringLiteral("edd"),
			QStringLiteral("cr")};
}

void ProcessFlowReportPanel::rebuildSummary(const SimStatistics& stats)
{
	m_summary->setText(
		(m_zh ? QStringLiteral("完成=%1  报废=%2  释放=%3  Makespan=%4s\n吞吐=%5 件/时  WIP均/峰=%6/%7  瓶颈=%8")
			  : QStringLiteral("done=%1  scrap=%2  released=%3  makespan=%4s\nthroughput=%5 /h  WIP avg/max=%6/%7  bottleneck=%8"))
			.arg(stats.completedJobs)
			.arg(stats.scrappedJobs)
			.arg(stats.releasedJobs)
			.arg(stats.makespan, 0, 'f', 1)
			.arg(stats.throughputPerHour, 0, 'f', 2)
			.arg(stats.avgWip, 0, 'f', 2)
			.arg(stats.maxWip, 0, 'f', 0)
			.arg(stats.bottleneckTitle.isEmpty() ? QString::number(stats.bottleneckNodeId) : stats.bottleneckTitle));

	m_machineTable->setRowCount(stats.machines.size());
	for (int r = 0; r < stats.machines.size(); ++r)
	{
		const MachineStat& m = stats.machines[r];
		m_machineTable->setItem(r, 0, new QTableWidgetItem(m.title.isEmpty() ? QString::number(m.nodeId) : m.title));
		m_machineTable->setItem(r, 1, new QTableWidgetItem(QString::number(m.utilization * 100.0, 'f', 1) + QLatin1Char('%')));
		m_machineTable->setItem(r, 2, new QTableWidgetItem(QString::number(m.busyTimeSec, 'f', 1)));
		m_machineTable->setItem(r, 3, new QTableWidgetItem(QString::number(m.blockedTimeSec, 'f', 1)));
		m_machineTable->setItem(r, 4, new QTableWidgetItem(QString::number(m.avgQueueLen, 'f', 2)));
	}

	m_bufferTable->setRowCount(stats.buffers.size());
	for (int r = 0; r < stats.buffers.size(); ++r)
	{
		const BufferStat& b = stats.buffers[r];
		m_bufferTable->setItem(r, 0, new QTableWidgetItem(b.title.isEmpty() ? QString::number(b.nodeId) : b.title));
		m_bufferTable->setItem(r, 1, new QTableWidgetItem(QString::number(b.avgInventory, 'f', 2)));
		m_bufferTable->setItem(r, 2, new QTableWidgetItem(QString::number(b.maxInventory, 'f', 0)));
		m_bufferTable->setItem(r, 3, new QTableWidgetItem(QString::number(b.fullCount)));
	}
}
