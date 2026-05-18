#include "AiLlmSettingsDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QSpinBox>
#include <QVBoxLayout>

AiLlmSettingsDialog::AiLlmSettingsDialog(QWidget* parent)
	: QDialog(parent)
{
	setWindowTitle(QStringLiteral("AI Assistant / LLM Settings"));
	setModal(true);
	resize(480, 420);
	buildUi();
	loadFromFileOrDefaults();
}

void AiLlmSettingsDialog::buildUi()
{
	auto* root = new QVBoxLayout(this);

	m_pathLabel = new QLabel(this);
	m_pathLabel->setWordWrap(true);
	m_pathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
	root->addWidget(m_pathLabel);

	auto* formBox = new QGroupBox(QStringLiteral("Connection"), this);
	auto* form = new QFormLayout(formBox);

	m_enabled = new QCheckBox(QStringLiteral("Enable LLM when rule parser fails"), formBox);
	m_ruleFirst = new QCheckBox(QStringLiteral("Try offline rule parser before LLM"), formBox);
	m_ruleFirst->setChecked(false);
	m_baseUrl = new QLineEdit(formBox);
	m_baseUrl->setPlaceholderText(QStringLiteral("https://api.openai.com/v1"));
	m_apiKey = new QLineEdit(formBox);
	m_apiKey->setEchoMode(QLineEdit::Password);
	m_apiKey->setPlaceholderText(QStringLiteral("Leave empty to use environment variable"));
	m_apiKeyEnv = new QLineEdit(formBox);
	m_model = new QLineEdit(formBox);
	m_timeoutMs = new QSpinBox(formBox);
	m_timeoutMs->setRange(5000, 300000);
	m_timeoutMs->setSingleStep(1000);
	m_timeoutMs->setSuffix(QStringLiteral(" ms"));
	m_temperature = new QDoubleSpinBox(formBox);
	m_temperature->setRange(0.0, 2.0);
	m_temperature->setSingleStep(0.1);
	m_temperature->setDecimals(2);

	form->addRow(QString(), m_enabled);
	form->addRow(QString(), m_ruleFirst);
	form->addRow(QStringLiteral("API base URL"), m_baseUrl);
	form->addRow(QStringLiteral("API key"), m_apiKey);
	form->addRow(QStringLiteral("API key env var"), m_apiKeyEnv);
	form->addRow(QStringLiteral("Model"), m_model);
	form->addRow(QStringLiteral("Timeout"), m_timeoutMs);
	form->addRow(QStringLiteral("Temperature"), m_temperature);

	root->addWidget(formBox);

	auto* hint = new QLabel(
		QStringLiteral("Config file is saved next to the application executable.\n"
		               "Compatible with OpenAI-style chat/completions APIs."),
		this);
	hint->setWordWrap(true);
	hint->setStyleSheet(QStringLiteral("color: palette(mid);"));
	root->addWidget(hint);

	m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	connect(m_buttons, &QDialogButtonBox::accepted, this, &AiLlmSettingsDialog::onAccepted);
	connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
	root->addWidget(m_buttons);
}

void AiLlmSettingsDialog::loadFromFileOrDefaults()
{
	m_pathLabel->setText(QStringLiteral("File: %1").arg(defaultAiConfigPath()));
	if (const auto loaded = loadAiLlmConfig())
		setConfig(*loaded);
	else
		setConfig(defaultAiLlmConfig());
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
		QMessageBox::warning(this, windowTitle(), QStringLiteral("API base URL is required when LLM is enabled."));
		return;
	}
	if (cfg.enabled && cfg.model.isEmpty())
	{
		QMessageBox::warning(this, windowTitle(), QStringLiteral("Model name is required when LLM is enabled."));
		return;
	}
	if (cfg.enabled && !cfg.hasApiKey())
	{
		QMessageBox::warning(this, windowTitle(),
			QStringLiteral("Set an API key or configure a non-empty environment variable when LLM is enabled."));
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
