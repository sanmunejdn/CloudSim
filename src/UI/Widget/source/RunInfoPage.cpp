/// @file RunInfoPage.cpp
/// @brief 运行信息页

#include "RunInfoPage.h"

#include "RunLogger.h"
#include "UiIconDecorators.h"

#include <QByteArray>
#include <QDateTime>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QStyle>
#include <QVBoxLayout>

namespace
{
std::string toUtf8StdString(const QString& text)
{
	const QByteArray utf8 = text.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}
} // namespace

RunInfoPage::RunInfoPage(QWidget* parent) : QWidget(parent)
{
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(4, 4, 4, 4);
	root->setSpacing(4);

	auto* toolRow = new QHBoxLayout;
	toolRow->setContentsMargins(0, 0, 0, 0);
	toolRow->addStretch(1);
	m_clearBtn = new QPushButton(QStringLiteral("Clear"), this);
	m_clearBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	toolRow->addWidget(m_clearBtn);
	root->addLayout(toolRow, 0);

	m_logEdit = new QPlainTextEdit(this);
	m_logEdit->setReadOnly(true);
	m_logEdit->setMaximumBlockCount(2000);
	m_logEdit->setMinimumHeight(36);
	m_logEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	root->addWidget(m_logEdit, 1);

	connect(m_clearBtn, &QPushButton::clicked, this, &RunInfoPage::clearLogs);
	UiIconDecorators::apply(m_clearBtn, UiIconId::ClearLog);
	m_clearBtn->setProperty("btnRole", QLatin1String("danger"));
	if (m_clearBtn->style())
	{
		m_clearBtn->style()->unpolish(m_clearBtn);
		m_clearBtn->style()->polish(m_clearBtn);
	}

	RunLogger::setUiSink(
		[this](RunLogger::LogLevel level, const std::string& message)
		{
			const QString levelText = QString::fromLatin1(RunLogger::levelName(level));
			const QString text = QString::fromUtf8(message.c_str());
			QMetaObject::invokeMethod(
				this, [this, levelText, text]() { appendLine(levelText, text); }, Qt::QueuedConnection);
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
	m_clearBtn->setText(useChinese ? QStringLiteral("清空") : QStringLiteral("Clear"));
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
