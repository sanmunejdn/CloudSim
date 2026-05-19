#include "AiLlmSettingsDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
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
void setFormLabel(QFormLayout* form, const int row, const QString& text)
{
	if (!form || row < 0 || row >= form->rowCount())
	{
		return;
	}
	if (QLabel* label = qobject_cast<QLabel*>(form->itemAt(row, QFormLayout::LabelRole)->widget()))
	{
		label->setText(text);
	}
}
} // namespace

AiLlmSettingsDialog::AiLlmSettingsDialog(QWidget* parent)
	: QDialog(parent)
{
	setModal(true);
	resize(480, 420);
	buildUi();
	loadFromFileOrDefaults();
	applyLanguage();
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

	m_enabled = new QCheckBox(QStringLiteral("Enable LLM when rule parser fails"), m_connectionGroup);
	m_ruleFirst = new QCheckBox(QStringLiteral("Try offline rule parser before LLM"), m_connectionGroup);
	m_ruleFirst->setChecked(false);
	m_baseUrl = new QLineEdit(m_connectionGroup);
	m_baseUrl->setPlaceholderText(QStringLiteral("https://api.openai.com/v1"));
	m_apiKey = new QLineEdit(m_connectionGroup);
	m_apiKey->setEchoMode(QLineEdit::Password);
	m_apiKey->setPlaceholderText(QStringLiteral("Leave empty to use environment variable"));
	m_apiKeyEnv = new QLineEdit(m_connectionGroup);
	m_model = new QLineEdit(m_connectionGroup);
	m_timeoutMs = new QSpinBox(m_connectionGroup);
	m_timeoutMs->setRange(5000, 300000);
	m_timeoutMs->setSingleStep(1000);
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

	m_hintLabel = new QLabel(
		QStringLiteral("Config file is saved next to the application executable.\n"
		               "Compatible with OpenAI-style chat/completions APIs."),
		this);
	m_hintLabel->setWordWrap(true);
	m_hintLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
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
	setWindowTitle(zh ? QStringLiteral("AI 助手 / 大模型设置") : QStringLiteral("AI Assistant / LLM Settings"));
	updatePathLabel();

	if (m_connectionGroup)
	{
		m_connectionGroup->setTitle(zh ? QStringLiteral("连接") : QStringLiteral("Connection"));
	}
	if (m_enabled)
	{
		m_enabled->setText(zh ? QStringLiteral("规则解析失败时启用大模型")
							  : QStringLiteral("Enable LLM when rule parser fails"));
	}
	if (m_ruleFirst)
	{
		m_ruleFirst->setText(zh ? QStringLiteral("调用大模型前先尝试离线规则解析")
							   : QStringLiteral("Try offline rule parser before LLM"));
	}
	if (m_apiKey)
	{
		m_apiKey->setPlaceholderText(zh ? QStringLiteral("留空则使用环境变量中的密钥")
										: QStringLiteral("Leave empty to use environment variable"));
	}
	if (m_form)
	{
		setFormLabel(m_form, 2, zh ? QStringLiteral("API 基础地址") : QStringLiteral("API base URL"));
		setFormLabel(m_form, 3, zh ? QStringLiteral("API 密钥") : QStringLiteral("API key"));
		setFormLabel(m_form, 4, zh ? QStringLiteral("API 密钥环境变量") : QStringLiteral("API key env var"));
		setFormLabel(m_form, 5, zh ? QStringLiteral("模型") : QStringLiteral("Model"));
		setFormLabel(m_form, 6, zh ? QStringLiteral("超时") : QStringLiteral("Timeout"));
		setFormLabel(m_form, 7, zh ? QStringLiteral("温度") : QStringLiteral("Temperature"));
	}
	if (m_hintLabel)
	{
		m_hintLabel->setText(zh
			? QStringLiteral("配置文件保存在应用程序可执行文件同目录。\n"
							 "兼容 OpenAI 风格的 chat/completions API。")
			: QStringLiteral("Config file is saved next to the application executable.\n"
							 "Compatible with OpenAI-style chat/completions APIs."));
	}
	if (m_buttons)
	{
		if (QPushButton* ok = m_buttons->button(QDialogButtonBox::Ok))
		{
			ok->setText(zh ? QStringLiteral("确定") : QStringLiteral("OK"));
		}
		if (QPushButton* cancel = m_buttons->button(QDialogButtonBox::Cancel))
		{
			cancel->setText(zh ? QStringLiteral("取消") : QStringLiteral("Cancel"));
		}
	}
}

void AiLlmSettingsDialog::updatePathLabel()
{
	if (!m_pathLabel)
	{
		return;
	}
	const QString prefix = m_useChinese ? QStringLiteral("文件：") : QStringLiteral("File: ");
	m_pathLabel->setText(prefix + defaultAiConfigPath());
}

void AiLlmSettingsDialog::loadFromFileOrDefaults()
{
	updatePathLabel();
	if (const auto loaded = loadAiLlmConfig())
	{
		setConfig(*loaded);
	}
	else
	{
		setConfig(defaultAiLlmConfig());
	}
}

AiLlmConfig AiLlmSettingsDialog::config() const
{
	AiLlmConfig cfg;
	cfg.enabled = m_enabled->isChecked();
	cfg.ruleParserFirst = m_ruleFirst->isChecked();
	cfg.baseUrl = m_baseUrl->text().trimmed();
	cfg.apiKey = m_apiKey->text().trimmed();
	cfg.apiKeyEnv = m_apiKeyEnv->text().trimmed();
	cfg.model = m_model->text().trimmed();
	cfg.timeoutMs = m_timeoutMs->value();
	cfg.temperature = m_temperature->value();
	return cfg;
}

void AiLlmSettingsDialog::setConfig(const AiLlmConfig& cfg)
{
	m_enabled->setChecked(cfg.enabled);
	m_ruleFirst->setChecked(cfg.ruleParserFirst);
	m_baseUrl->setText(cfg.baseUrl);
	m_apiKey->setText(cfg.apiKey);
	m_apiKeyEnv->setText(cfg.apiKeyEnv);
	m_model->setText(cfg.model);
	m_timeoutMs->setValue(std::max(5000, cfg.timeoutMs));
	m_temperature->setValue(cfg.temperature);
}

QString AiLlmSettingsDialog::configFilePath() const
{
	return defaultAiConfigPath();
}

void AiLlmSettingsDialog::onAccepted()
{
	const AiLlmConfig cfg = config();
	if (cfg.enabled && cfg.baseUrl.isEmpty())
	{
		QMessageBox::warning(this, windowTitle(),
			m_useChinese ? QStringLiteral("启用大模型时必须填写 API 基础地址。")
						 : QStringLiteral("API base URL is required when LLM is enabled."));
		return;
	}
	if (cfg.enabled && cfg.model.isEmpty())
	{
		QMessageBox::warning(this, windowTitle(),
			m_useChinese ? QStringLiteral("启用大模型时必须填写模型名称。")
						 : QStringLiteral("Model name is required when LLM is enabled."));
		return;
	}
	if (cfg.enabled && !cfg.hasApiKey())
	{
		QMessageBox::warning(this, windowTitle(),
			m_useChinese ? QStringLiteral("启用大模型时请填写 API 密钥，或配置非空的环境变量名。")
						 : QStringLiteral(
							   "Set an API key or configure a non-empty environment variable when LLM is enabled."));
		return;
	}

	QString err;
	if (!saveAiLlmConfig(cfg, QString(), &err))
	{
		QMessageBox::critical(this, windowTitle(), err);
		return;
	}
	accept();
}
