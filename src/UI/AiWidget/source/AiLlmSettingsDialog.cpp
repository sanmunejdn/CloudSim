#include "AiLlmSettingsDialog.h"

#include "AiConfigDefaults.h"
#include "IAiAssistantHost.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace
{
QString configPath()
{
	return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("ai_config.json"));
}

void setFormLabel(QFormLayout* form, const int row, const QString& text)
{
	if (!form || row < 0 || row >= form->rowCount())
		return;
	if (QLabel* label = qobject_cast<QLabel*>(form->itemAt(row, QFormLayout::LabelRole)->widget()))
		label->setText(text);
}
}

AiLlmSettingsDialog::AiLlmSettingsDialog(QWidget* parent)
	: QDialog(parent)
{
	setModal(true);
	resize(480, 420);
	buildUi();
	loadFromFileOrDefaults();
	applyLanguage();
}

void AiLlmSettingsDialog::setAiHost(IAiAssistantHost* host)
{
	m_aiHost = host;
	loadFromFileOrDefaults();
}

void AiLlmSettingsDialog::buildUi()
{
	auto* root = new QVBoxLayout(this);
	m_pathLabel = new QLabel(this);
	m_pathLabel->setWordWrap(true);
	m_pathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
	root->addWidget(m_pathLabel);

	m_connectionGroup = new QGroupBox(QStringLiteral("Connection"), this);
	m_form = new QFormLayout(m_connectionGroup);
	m_enabled = new QCheckBox(m_connectionGroup);
	m_ruleFirst = new QCheckBox(m_connectionGroup);
	m_baseUrl = new QLineEdit(m_connectionGroup);
	m_apiKey = new QLineEdit(m_connectionGroup);
	m_apiKey->setEchoMode(QLineEdit::Password);
	m_apiKeyEnv = new QLineEdit(m_connectionGroup);
	m_model = new QLineEdit(m_connectionGroup);
	m_timeoutMs = new QSpinBox(m_connectionGroup);
	m_timeoutMs->setRange(5000, 300000);
	m_timeoutMs->setSuffix(QStringLiteral(" ms"));
	m_temperature = new QDoubleSpinBox(m_connectionGroup);
	m_temperature->setRange(0.0, 2.0);
	m_temperature->setSingleStep(0.1);
	m_temperature->setDecimals(2);

	m_form->addRow(QString(), m_enabled);
	m_form->addRow(QString(), m_ruleFirst);
	m_form->addRow(QStringLiteral("API base URL"), m_baseUrl);
	m_form->addRow(QStringLiteral("API key"), m_apiKey);
	m_form->addRow(QStringLiteral("API key env var"), m_apiKeyEnv);
	m_form->addRow(QStringLiteral("Model"), m_model);
	m_form->addRow(QStringLiteral("Timeout"), m_timeoutMs);
	m_form->addRow(QStringLiteral("Temperature"), m_temperature);
	root->addWidget(m_connectionGroup);

	m_hintLabel = new QLabel(this);
	m_hintLabel->setWordWrap(true);
	root->addWidget(m_hintLabel);

	m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	connect(m_buttons, &QDialogButtonBox::accepted, this, &AiLlmSettingsDialog::onAccepted);
	connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
	root->addWidget(m_buttons);
}

void AiLlmSettingsDialog::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	applyLanguage();
}

void AiLlmSettingsDialog::applyLanguage()
{
	const bool zh = m_useChinese;
	setWindowTitle(zh ? QStringLiteral("AI 助手设置") : QStringLiteral("AI Assistant Settings"));
	updatePathLabel();
	if (m_connectionGroup)
		m_connectionGroup->setTitle(zh ? QStringLiteral("云端大模型（可选）") : QStringLiteral("Remote LLM (optional)"));
	if (m_enabled)
		m_enabled->setText(zh ? QStringLiteral("启用云端大模型回退") : QStringLiteral("Enable remote LLM fallback"));
	if (m_ruleFirst)
		m_ruleFirst->setText(zh ? QStringLiteral("本地解析优先使用规则") : QStringLiteral("Prefer rules in local parser chain"));
	if (m_hintLabel)
		m_hintLabel->setText(zh
			? QStringLiteral("默认使用本地 Ollama（domains 配置）。此处仅配置可选云端 API。")
			: QStringLiteral("Local Ollama (domains) is default. Configure optional remote API here."));
	if (m_buttons)
	{
		if (QPushButton* ok = m_buttons->button(QDialogButtonBox::Ok))
			ok->setText(zh ? QStringLiteral("确定") : QStringLiteral("OK"));
		if (QPushButton* cancel = m_buttons->button(QDialogButtonBox::Cancel))
			cancel->setText(zh ? QStringLiteral("取消") : QStringLiteral("Cancel"));
	}
}

void AiLlmSettingsDialog::updatePathLabel()
{
	if (m_pathLabel)
	{
		const QString prefix = m_useChinese ? QStringLiteral("文件：") : QStringLiteral("File: ");
		m_pathLabel->setText(prefix + configPath());
	}
}

void AiLlmSettingsDialog::loadFromFileOrDefaults()
{
	updatePathLabel();
	AiConfigDto cfg = defaultAiConfigDto();
	if (m_aiHost)
	{
		if (const auto loaded = m_aiHost->loadConfig())
			cfg = *loaded;
	}
	setConfig(cfg);
}

AiConfigDto AiLlmSettingsDialog::config() const
{
	AiConfigDto cfg = defaultAiConfigDto();
	if (m_aiHost)
	{
		if (const auto loaded = m_aiHost->loadConfig())
			cfg = *loaded;
	}
	cfg.remoteLlm.enabled = m_enabled->isChecked();
	cfg.remoteLlm.baseUrl = m_baseUrl->text().trimmed();
	cfg.remoteLlm.apiKey = m_apiKey->text().trimmed();
	cfg.remoteLlm.apiKeyEnv = m_apiKeyEnv->text().trimmed();
	cfg.remoteLlm.model = m_model->text().trimmed();
	cfg.remoteLlm.timeoutMs = m_timeoutMs->value();
	cfg.remoteLlm.temperature = m_temperature->value();
	if (m_ruleFirst->isChecked() && !cfg.domains.empty())
		cfg.domains[0].parserPriority = QStringList{
			QStringLiteral("rules"),
			QStringLiteral("local"),
			QStringLiteral("remote")
		};
	return cfg;
}

void AiLlmSettingsDialog::setConfig(const AiConfigDto& cfg)
{
	m_enabled->setChecked(cfg.remoteLlm.enabled);
	m_baseUrl->setText(cfg.remoteLlm.baseUrl);
	m_apiKey->setText(cfg.remoteLlm.apiKey);
	m_apiKeyEnv->setText(cfg.remoteLlm.apiKeyEnv);
	m_model->setText(cfg.remoteLlm.model);
	m_timeoutMs->setValue(std::max(5000, cfg.remoteLlm.timeoutMs));
	m_temperature->setValue(cfg.remoteLlm.temperature);
	const bool ruleFirst = !cfg.domains.empty() && cfg.domains[0].parserPriority.value(0) == QStringLiteral("rules");
	m_ruleFirst->setChecked(ruleFirst);
}

void AiLlmSettingsDialog::onAccepted()
{
	if (!m_aiHost)
	{
		reject();
		return;
	}
	AiConfigDto cfg = config();
	QString err;
	if (!m_aiHost->saveConfig(cfg, &err))
	{
		QMessageBox::critical(this, windowTitle(), err);
		return;
	}
	accept();
}
