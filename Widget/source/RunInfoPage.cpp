#include "RunInfoPage.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

RunInfoPage::RunInfoPage(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	auto* toolRow = new QHBoxLayout;
	toolRow->addStretch(1);
	m_clearBtn = new QPushButton(QStringLiteral("Clear"), this);
	toolRow->addWidget(m_clearBtn);
	root->addLayout(toolRow);

	m_logEdit = new QPlainTextEdit(this);
	m_logEdit->setReadOnly(true);
	m_logEdit->setMaximumBlockCount(2000);
	root->addWidget(m_logEdit, 1);

	connect(m_clearBtn, &QPushButton::clicked, this, &RunInfoPage::clearLogs);
}

void RunInfoPage::appendInfo(const QString& text)
{
	appendLine(QStringLiteral("INFO"), text);
}

void RunInfoPage::appendWarning(const QString& text)
{
	appendLine(QStringLiteral("WARN"), text);
}

void RunInfoPage::appendError(const QString& text)
{
	appendLine(QStringLiteral("ERROR"), text);
}

void RunInfoPage::clearLogs()
{
	if (m_logEdit)
	{
		m_logEdit->clear();
	}
}

void RunInfoPage::setUiLanguage(bool useChinese)
{
	if (!m_clearBtn)
	{
		return;
	}
	m_clearBtn->setText(useChinese ? QStringLiteral("\u6E05\u7A7A") : QStringLiteral("Clear"));
}

void RunInfoPage::appendLine(const QString& level, const QString& text)
{
	if (!m_logEdit)
	{
		return;
	}
	const QString time = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss"));
	m_logEdit->appendPlainText(QStringLiteral("[%1] [%2] %3").arg(time, level, text));
}
