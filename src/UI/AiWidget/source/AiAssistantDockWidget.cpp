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
	m_input->setPlaceholderText(QStringLiteral("e.g. create box 100x50x100 mm / 生成长方体，长宽高为100x50x100 mm"));
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

	setUseChinese(m_useChinese);
	appendSystemMessage(m_useChinese
		? QStringLiteral("AI 助手：优先使用大模型（设置）。可选离线规则。单位：mm。")
		: QStringLiteral("AI assistant: LLM first (Settings). Offline rules optional. Units: mm."));
}

void AiAssistantDockWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	m_settingsBtn->setText(chinese ? QStringLiteral("设置") : QStringLiteral("Settings"));
	m_sendBtn->setText(chinese ? QStringLiteral("发送") : QStringLiteral("Send"));
	m_settingsBtn->setToolTip(chinese ? QStringLiteral("配置大模型（ai_config.json）")
									  : QStringLiteral("Configure LLM (ai_config.json)"));
	m_input->setPlaceholderText(
		chinese ? QStringLiteral("例如：生成长方体，长宽高为 100x50x100 mm / create box 100x50x100 mm")
				: QStringLiteral("e.g. create box 100x50x100 mm / 生成长方体，长宽高为100x50x100 mm"));
}

void AiAssistantDockWidget::appendUserMessage(const QString& text)
{
	const QString who = m_useChinese ? QStringLiteral("你") : QStringLiteral("You");
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped()));
}

void AiAssistantDockWidget::appendAssistantMessage(const QString& text)
{
	const QString who = m_useChinese ? QStringLiteral("助手") : QStringLiteral("Assistant");
	m_history->append(QStringLiteral("<b>%1:</b> %2").arg(who, text.toHtmlEscaped()));
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
	dlg.setUseChinese(m_useChinese);
	if (dlg.exec() != QDialog::Accepted)
		return;
	const auto cfg = loadAiLlmConfig();
	if (cfg && cfg->enabled && cfg->hasApiKey())
	{
		const QString yesNo = m_useChinese ? QStringLiteral("是") : QStringLiteral("yes");
		const QString noWord = m_useChinese ? QStringLiteral("否") : QStringLiteral("no");
		appendSystemMessage(m_useChinese
			? QStringLiteral("已启用大模型（%1）。优先规则解析：%2。")
				  .arg(cfg->model, cfg->ruleParserFirst ? yesNo : noWord)
			: QStringLiteral("LLM enabled (%1). Rule parser first: %2.")
				  .arg(cfg->model, cfg->ruleParserFirst ? yesNo : noWord));
	}
	else
	{
		appendSystemMessage(m_useChinese ? QStringLiteral("设置已保存。大模型未启用或仅使用离线规则。")
										 : QStringLiteral("Settings saved. LLM is off or uses rules only."));
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
