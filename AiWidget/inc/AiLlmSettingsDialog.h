#pragma once

#include <QDialog>

#include "aiwidget_global.h"
#include "AiLlmConfig.h"

class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QSpinBox;

/// Edit and save `ai_config.json` next to the executable.
class AIWIDGET_EXPORT AiLlmSettingsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit AiLlmSettingsDialog(QWidget* parent = nullptr);

	AiLlmConfig config() const;
	void setConfig(const AiLlmConfig& config);

	QString configFilePath() const;

private slots:
	void onAccepted();

private:
	void buildUi();
	void loadFromFileOrDefaults();

	QLabel* m_pathLabel = nullptr;
	QCheckBox* m_enabled = nullptr;
	QCheckBox* m_ruleFirst = nullptr;
	QLineEdit* m_baseUrl = nullptr;
	QLineEdit* m_apiKey = nullptr;
	QLineEdit* m_apiKeyEnv = nullptr;
	QLineEdit* m_model = nullptr;
	QSpinBox* m_timeoutMs = nullptr;
	QDoubleSpinBox* m_temperature = nullptr;
	QDialogButtonBox* m_buttons = nullptr;
};
