/// @file ProcessFlowPageWidget.cpp
/// @brief 中央流程页实现

#include "ProcessFlowPageWidget.h"

#include "ProcessFlowCanvasWidget.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

ProcessFlowPageWidget::ProcessFlowPageWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);

	auto* toolbar = new QWidget(this);
	auto* bar = new QHBoxLayout(toolbar);
	bar->setContentsMargins(8, 6, 8, 6);
	bar->setSpacing(8);

	m_connectCheck = new QCheckBox(QStringLiteral("连线模式"), toolbar);
	m_gridCheck = new QCheckBox(QStringLiteral("显示网格"), toolbar);
	m_gridCheck->setChecked(true);

	m_deleteBtn = new QPushButton(QStringLiteral("删除选中"), toolbar);
	m_layoutBtn = new QPushButton(QStringLiteral("自动排版"), toolbar);
	m_fitBtn = new QPushButton(QStringLiteral("适应窗口"), toolbar);
	m_exportBtn = new QPushButton(QStringLiteral("导出 JSON"), toolbar);

	bar->addWidget(m_connectCheck);
	bar->addWidget(m_gridCheck);
	bar->addWidget(m_deleteBtn);
	bar->addWidget(m_layoutBtn);
	bar->addWidget(m_fitBtn);
	bar->addWidget(m_exportBtn);
	bar->addStretch(1);

	m_canvas = new ProcessFlowCanvasWidget(this);

	connect(m_connectCheck, &QCheckBox::toggled, m_canvas, &ProcessFlowCanvasWidget::setConnectionMode);
	connect(m_gridCheck, &QCheckBox::toggled, m_canvas, &ProcessFlowCanvasWidget::setGridVisible);
	connect(m_deleteBtn, &QPushButton::clicked, m_canvas, &ProcessFlowCanvasWidget::removeSelectedItem);
	connect(m_layoutBtn, &QPushButton::clicked, m_canvas, &ProcessFlowCanvasWidget::autoLayout);
	connect(m_fitBtn, &QPushButton::clicked, m_canvas, &ProcessFlowCanvasWidget::fitToView);
	connect(m_exportBtn, &QPushButton::clicked, this,
			[this]()
			{
				const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出流程 JSON"), QString(),
																 QStringLiteral("JSON (*.json)"));
				if (path.isEmpty())
				{
					return;
				}
				if (!m_canvas->exportJson(path))
				{
					QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("无法写入文件。"));
				}
			});

	root->addWidget(toolbar);
	root->addWidget(m_canvas, 1);
}

void ProcessFlowPageWidget::applyLanguage(bool useChinese)
{
	if (m_connectCheck)
	{
		m_connectCheck->setText(useChinese ? QStringLiteral("连线模式") : QStringLiteral("Connect Mode"));
	}
	if (m_gridCheck)
	{
		m_gridCheck->setText(useChinese ? QStringLiteral("显示网格") : QStringLiteral("Show Grid"));
	}
	if (m_deleteBtn)
	{
		m_deleteBtn->setText(useChinese ? QStringLiteral("删除选中") : QStringLiteral("Delete"));
	}
	if (m_layoutBtn)
	{
		m_layoutBtn->setText(useChinese ? QStringLiteral("自动排版") : QStringLiteral("Auto Layout"));
	}
	if (m_fitBtn)
	{
		m_fitBtn->setText(useChinese ? QStringLiteral("适应窗口") : QStringLiteral("Fit View"));
	}
	if (m_exportBtn)
	{
		m_exportBtn->setText(useChinese ? QStringLiteral("导出 JSON") : QStringLiteral("Export JSON"));
	}
}
