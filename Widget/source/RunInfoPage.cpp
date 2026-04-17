#include "RunInfoPage.h"

#include "RunLogger.h"

#include <QByteArray>
#include <QDateTime>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace
{
std::string toUtf8StdString(const QString& text)
{
	const QByteArray utf8 = text.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}
} // namespace

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

	RunLogger::setUiSink([this](RunLogger::LogLevel level, const std::string& message) {
		const QString levelText = QString::fromLatin1(RunLogger::levelName(level));
		const QString text = QString::fromUtf8(message.c_str());
		QMetaObject::invokeMethod(
			this,
			[this, levelText, text]() { appendLine(levelText, text); },
			Qt::QueuedConnection);
	});
}

RunInfoPage::~RunInfoPage()
{
	RunLogger::clearUiSink();
}

void RunInfoPage::appendInfo(const QString& text)
{
	RunLogger::info(toUtf8StdString(text));
}

void RunInfoPage::appendWarning(const QString& text)
{
	RunLogger::warn(toUtf8StdString(text));
}

void RunInfoPage::appendError(const QString& text)
{
	RunLogger::error(toUtf8StdString(text));
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
