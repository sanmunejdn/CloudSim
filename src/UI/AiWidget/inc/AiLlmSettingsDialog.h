#pragma once

#include <QString>
#include <QDialog>

#include "aiwidget_global.h"
#include "AiLlmConfig.h"

class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QFormLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QSpinBox;

/// 编辑并保存可执行文件旁 ai_config.json
class AIWIDGET_EXPORT AiLlmSettingsDialog : public QDialog
{
	Q_OBJECT

public:
	explicit AiLlmSettingsDialog(QWidget* parent = nullptr);

	AiLlmConfig config() const;
	void setConfig(const AiLlmConfig& config);
	void setUseChinese(bool chinese);

	QString configFilePath() const;

private slots:
	void onAccepted();

private:
	void buildUi();
	void loadFromFileOrDefaults();
	void applyLanguage();
	void updatePathLabel();

	bool m_useChinese = true;
	QLabel* m_pathLabel = nullptr;
	QLabel* m_hintLabel = nullptr;
	QGroupBox* m_connectionGroup = nullptr;
	QFormLayout* m_form = nullptr;
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
