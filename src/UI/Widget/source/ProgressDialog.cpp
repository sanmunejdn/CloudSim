/// @file ProgressDialog.cpp
/// @brief 通用进度对话框

#include "ProgressDialog.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QtGlobal>
#include <QVBoxLayout>

ProgressDialog::ProgressDialog(QWidget* parent) : QDialog(parent)
{
	setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
	setModal(true);
	setMinimumWidth(420);
	setWindowTitle(QStringLiteral("进度"));

	auto* root = new QVBoxLayout(this);
	m_titleLabel = new QLabel(this);
	m_titleLabel->setWordWrap(true);
	QFont titleFont = m_titleLabel->font();
	titleFont.setBold(true);
	m_titleLabel->setFont(titleFont);
	root->addWidget(m_titleLabel);

	m_statusLabel = new QLabel(this);
	m_statusLabel->setWordWrap(true);
	root->addWidget(m_statusLabel);

	m_bar = new QProgressBar(this);
	m_bar->setRange(0, 1000);
	m_bar->setValue(0);
	m_bar->setTextVisible(true);
	root->addWidget(m_bar);

	m_extraLabel = new QLabel(this);
	m_extraLabel->setVisible(false);
	root->addWidget(m_extraLabel);

	auto* row = new QHBoxLayout();
	row->addStretch(1);
	m_cancelBtn = new QPushButton(this);
	m_cancelBtn->setEnabled(false);
	row->addWidget(m_cancelBtn);
	root->addLayout(row);

	connect(m_cancelBtn, &QPushButton::clicked, this, &ProgressDialog::cancelRequested);
	setChinese(true);
}

void ProgressDialog::setChinese(bool useChinese)
{
	m_useChinese = useChinese;
	m_cancelBtn->setText(useChinese ? QStringLiteral("取消") : QStringLiteral("Cancel"));
	setWindowTitle(useChinese ? QStringLiteral("进度") : QStringLiteral("Progress"));
}

void ProgressDialog::setJobTitle(const QString& title)
{
	m_titleLabel->setText(title);
}

void ProgressDialog::setStatusText(const QString& text)
{
	m_statusLabel->setText(text);
	m_statusLabel->setVisible(!text.isEmpty());
}

void ProgressDialog::setFraction(double fraction01)
{
	if (fraction01 < 0.0)
	{
		setIndeterminate(true);
		return;
	}
	setIndeterminate(false);
	const int v = static_cast<int>(qBound(0.0, 1.0, fraction01) * 1000.0 + 0.5);
	m_bar->setValue(v);
}

void ProgressDialog::setIndeterminate(bool on)
{
	if (on)
	{
		m_bar->setRange(0, 0);
	}
	else if (m_bar->maximum() == 0)
	{
		m_bar->setRange(0, 1000);
	}
}

void ProgressDialog::setCancelEnabled(bool enabled)
{
	m_cancelBtn->setEnabled(enabled);
	m_cancelBtn->setVisible(enabled);
}

void ProgressDialog::setExtraJobsHint(int otherCount)
{
	if (otherCount <= 0)
	{
		m_extraLabel->clear();
		m_extraLabel->setVisible(false);
		return;
	}
	m_extraLabel->setVisible(true);
	m_extraLabel->setText(m_useChinese ? QStringLiteral("另有 %1 个后台任务…").arg(otherCount)
									   : QStringLiteral("%1 more background job(s)…").arg(otherCount));
}
