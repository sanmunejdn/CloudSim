#include "AiAssistantDockWidget.h"

#include "AiDomainTypes.h"
#include "AiLlmSettingsDialog.h"
#include "IAiAssistantHost.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTextBrowser>
#include <QVBoxLayout>

AiAssistantDockWidget::AiAssistantDockWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);

	m_domainCombo = new QComboBox(this);
	m_domainCombo->addItem(QStringLiteral("Auto"), AiDomainIds::autoDomain());
	m_domainCombo->addItem(QStringLiteral("Create mesh"), AiDomainIds::meshCreate());
	m_domainCombo->addItem(QStringLiteral("Compose (boolean)"), AiDomainIds::meshCompose());
	m_domainCombo->addItem(QStringLiteral("Geometry recognize"), AiDomainIds::geometryRecognize());
	root->addWidget(m_domainCombo);

	m_history = new QTextBrowser(this);
	m_history->setOpenExternalLinks(false);
	m_history->setMinimumHeight(160);
	root->addWidget(m_history, 1);

	auto* row = new QHBoxLayout;
	m_input = new QLineEdit(this);
	m_settingsBtn = new QPushButton(this);
	connect(m_settingsBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSettingsClicked);
	m_sendBtn = new QPushButton(this);
	connect(m_sendBtn, &QPushButton::clicked, this, &AiAssistantDockWidget::onSendClicked);
	connect(m_input, &QLineEdit::returnPressed, this, &AiAssistantDockWidget::onSendClicked);
	row->addWidget(m_input, 1);
	row->addWidget(m_settingsBtn);
	row->addWidget(m_sendBtn);
	root->addLayout(row);

	setUseChinese(m_useChinese);
	appendSystemMessage(m_useChinese
		? QStringLiteral("AI 助手：默认本地模型 + 规则。单位 mm。")
		: QStringLiteral("AI assistant: local models + rules. Units: mm."));
}

void AiAssistantDockWidget::setAiHost(IAiAssistantHost* host)
{
	m_aiHost = host;
}

QString AiAssistantDockWidget::selectedDomainId() const
{
	if (!m_domainCombo)
		return AiDomainIds::autoDomain();
	return m_domainCombo->currentData().toString();
}

void AiAssistantDockWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	m_settingsBtn->setText(chinese ? QStringLiteral("设置") : QStringLiteral("Settings"));
	m_sendBtn->setText(chinese ? QStringLiteral("发送") : QStringLiteral("Send"));
	m_domainCombo->setItemText(0, chinese ? QStringLiteral("自动") : QStringLiteral("Auto"));
	m_domainCombo->setItemText(1, chinese ? QStringLiteral("创建网格") : QStringLiteral("Create mesh"));
	m_domainCombo->setItemText(2, chinese ? QStringLiteral("几何识别") : QStringLiteral("Geometry recognize"));
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
	m_domainCombo->setEnabled(!busy);
}

void AiAssistantDockWidget::onSettingsClicked()
{
	AiLlmSettingsDialog dlg(window());
	dlg.setUseChinese(m_useChinese);
	dlg.setAiHost(m_aiHost);
	if (dlg.exec() == QDialog::Accepted)
		appendSystemMessage(m_useChinese ? QStringLiteral("设置已保存。") : QStringLiteral("Settings saved."));
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
