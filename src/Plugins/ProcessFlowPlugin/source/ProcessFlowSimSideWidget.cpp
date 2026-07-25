/// @file ProcessFlowSimSideWidget.cpp
/// @brief 右侧 JobSet / 报表竖向分栏

#include "ProcessFlowSimSideWidget.h"

#include "ProcessFlowJobSetPanel.h"
#include "ProcessFlowReportPanel.h"
#include "ProcessFlowUiStyle.h"

#include <QFrame>
#include <QSplitter>
#include <QVBoxLayout>

namespace
{
QFrame* makeCard(QWidget* parent)
{
	auto* card = new QFrame(parent);
	card->setObjectName(QStringLiteral("ProcessFlowCard"));
	card->setFrameShape(QFrame::NoFrame);
	return card;
}
} // namespace

ProcessFlowSimSideWidget::ProcessFlowSimSideWidget(QWidget* parent) : QWidget(parent)
{
	setObjectName(QStringLiteral("ProcessFlowSideRoot"));
	setStyleSheet(processFlowSideChromeStyle());

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(10, 10, 10, 10);
	layout->setSpacing(10);

	m_jobSetPanel = new ProcessFlowJobSetPanel(this);
	m_reportPanel = new ProcessFlowReportPanel(this);

	auto* jobCard = makeCard(this);
	auto* jobLayout = new QVBoxLayout(jobCard);
	jobLayout->setContentsMargins(12, 10, 12, 10);
	jobLayout->addWidget(m_jobSetPanel);

	auto* reportCard = makeCard(this);
	auto* reportLayout = new QVBoxLayout(reportCard);
	reportLayout->setContentsMargins(12, 10, 12, 10);
	reportLayout->addWidget(m_reportPanel);

	auto* splitter = new QSplitter(Qt::Vertical, this);
	splitter->setChildrenCollapsible(false);
	splitter->addWidget(jobCard);
	splitter->addWidget(reportCard);
	splitter->setStretchFactor(0, 1);
	splitter->setStretchFactor(1, 2);
	layout->addWidget(splitter);
}

void ProcessFlowSimSideWidget::applyLanguage(bool useChinese)
{
	if (m_jobSetPanel)
	{
		m_jobSetPanel->applyLanguage(useChinese);
	}
	if (m_reportPanel)
	{
		m_reportPanel->applyLanguage(useChinese);
	}
}
