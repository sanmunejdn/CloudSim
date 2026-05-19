#include "AiAssistantDockWidget.h"

#include "AiLlmConfig.h"

#include <optional>
#include "AiLlmSettingsDialog.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QVBoxLayout>

AiAssistantDockWidget::AiAssistantDockWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	m_history = new QTextBrowser(this);
	m_history->setOpenExternalLinks(false);
	m_history->setMinimumHeight(160);
	root->addWidget(m_history, 1);

	auto* row = new QHBoxLayout;
	m_input = new QLineEdit(this);
	m_input->setPlaceholderText(QStringLiteral("e.g. create box 100x50x100 mm / ���ɳ����壬������Ϊ100x50x100 mm"));
	connect(m_input, &QLineEdit::returnPressed, this, &AiAssistantDockWidget::onSendClicked);
	m_settingsBtn = new QPushButton(QStringLiteral("Settings"), this);
	m_settingsBtn->setToolTip(QStringLiteral("Configure LLM (ai_config.json)"));
	connect(m_settingsBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSettingsClicked);
	m_sendBtn = new QPushButton(QStringLiteral("Send"), this);
	connect(m_sendBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSendClicked);
	row->addWidget(m_input, 1);
	row->addWidget(m_settingsBtn);
	row->addWidget(m_sendBtn);
	root->addLayout(row);

	appendSystemMessage(QStringLiteral("AI assistant: LLM first (Settings). Offline rules optional. Units: mm."));
}

void AiAssistantDockWidget::appendUserMessage(const QString& text)
{
	m_history->append(QStringLiteral("<b>You:</b> %1").arg(text.toHtmlEscaped()));
}

void AiAssistantDockWidget::appendAssistantMessage(const QString& text)
{
	m_history->append(QStringLiteral("<b>Assistant:</b> %1").arg(text.toHtmlEscaped()));
}

void AiAssistantDockWidget::appendSystemMessage(const QString& text)
{
	m_history->append(QStringLiteral("<i>%1</i>").arg(text.toHtmlEscaped()));
}

void AiAssistantDockWidget::setBusy(bool busy)
{
	m_sendBtn->setEnabled(!busy);
	m_settingsBtn->setEnabled(!busy);
	m_input->setEnabled(!busy);
}

void AiAssistantDockWidget::onSettingsClicked()
{
	AiLlmSettingsDialog dlg(window());
	if (dlg.exec() != QDialog::Accepted)
		return;
	const auto cfg = loadAiLlmConfig();
	if (cfg && cfg->enabled && cfg->hasApiKey())
	{
		appendSystemMessage(QStringLiteral("LLM enabled (%1). Rule parser first: %2.")
			.arg(cfg->model, cfg->ruleParserFirst ? QStringLiteral("yes") : QStringLiteral("no")));
	}
	else
	{
		appendSystemMessage(QStringLiteral("Settings saved. LLM is off or uses rules only."));
	}
}

void AiAssistantDockWidget::onSendClicked()
{
	const QString t = m_input->text().trimmed();
	if (t.isEmpty())
		return;
	m_input->clear();
	appendUserMessage(t);
	emit messageSubmitted(t);
}
